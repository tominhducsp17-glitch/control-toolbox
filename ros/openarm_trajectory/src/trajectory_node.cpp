// Node 1 "trajectory": plays a joint trajectory and publishes the target positions (the slow source, 100 Hz).
//
//   in : joint_states   (sensor_msgs/JointState)  measured pose, used to start from where the arm is
//   out: joint_commands (sensor_msgs/JointState)  position = q_target only (no velocity), rate_hz; dq_d and
//                                                  ddq_d come from the controller's reference filter
//   out: trajectory/phase (std_msgs/String)       wait_state | approach | track | return | hold
//
// Plan: wait for joint_states -> quintic approach from the measured pose to the first row (peak speed
// <= approach_speed) -> the trajectory file -> quintic return to the measured start pose -> hold it.
// lead_s: every message carries the plan at t + lead_s (targets known ahead, as an action chunk), which cancels
// the lag ~4 / filter_wn of the controller's reference filter.
// File: CSV with a header and columns t, q1..qN[, dq1..dqN[, ddq1..ddqN]]; only the positions are used.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Vec = Eigen::VectorXd;

struct Table
{
  std::vector<double> t;
  std::vector<Vec> q;
};

Table loadCsv(const std::string & path, std::size_t n)
{
  std::ifstream file(path);
  if (!file) {throw std::runtime_error("cannot open trajectory file " + path);}
  Table table;
  std::string line;
  std::getline(file, line);  // header
  while (std::getline(file, line)) {
    if (line.empty()) {continue;}
    std::vector<double> v;
    std::stringstream in(line);
    std::string cell;
    while (std::getline(in, cell, ',')) {v.push_back(std::stod(cell));}
    if (v.size() != 1 + n && v.size() != 1 + 2 * n && v.size() != 1 + 3 * n) {
      throw std::runtime_error("trajectory row has " + std::to_string(v.size()) + " columns, expected 1 + N, 1 + 2N or 1 + 3N");
    }
    if (!table.t.empty() && !(v[0] > table.t.back())) {throw std::runtime_error("trajectory time must increase");}
    table.t.push_back(v[0]);
    table.q.push_back(Eigen::Map<Vec>(v.data() + 1, static_cast<Eigen::Index>(n)));
  }
  if (table.t.empty() || std::abs(table.t.front()) > 1e-9) {throw std::runtime_error("trajectory must start at t = 0");}
  return table;
}

// Quintic a -> b over T: position at t.
Vec quintic(const Vec & a, const Vec & b, double T, double t)
{
  const double s = std::clamp(t / T, 0.0, 1.0);
  return a + s * s * s * (10 - 15 * s + 6 * s * s) * (b - a);
}

double quinticDuration(const Vec & a, const Vec & b, double speed, double min_s)
{
  return std::max(min_s, 1.875 * (b - a).cwiseAbs().maxCoeff() / speed);
}
}  // namespace

class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode()
  : Node("trajectory")
  {
    // Every parameter comes from the YAML loaded at start (config/trajectory.yaml); none has a
    // default in the code.
    joints_ = required("joint_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY).as_string_array();
    const auto file = required("trajectory_file", rclcpp::ParameterType::PARAMETER_STRING).as_string();
    rate_hz_ = required("rate_hz", rclcpp::ParameterType::PARAMETER_DOUBLE).as_double();
    approach_speed_ = required("approach_speed", rclcpp::ParameterType::PARAMETER_DOUBLE).as_double();
    min_move_s_ = required("min_move_s", rclcpp::ParameterType::PARAMETER_DOUBLE).as_double();
    return_to_start_ = required("return_to_start", rclcpp::ParameterType::PARAMETER_BOOL).as_bool();
    lead_s_ = required("lead_s", rclcpp::ParameterType::PARAMETER_DOUBLE).as_double();
    if (!(rate_hz_ > 0.0) || lead_s_ < 0.0) {throw std::invalid_argument("rate_hz must be > 0 and lead_s >= 0");}
    const auto command_topic = required("command_topic", rclcpp::ParameterType::PARAMETER_STRING).as_string();
    const auto state_topic = required("state_topic", rclcpp::ParameterType::PARAMETER_STRING).as_string();
    if (file.empty()) {throw std::invalid_argument("parameter trajectory_file is required");}
    table_ = loadCsv(file, joints_.size());
    RCLCPP_INFO(get_logger(), "trajectory %s: %zu rows, %.2f s, %zu joints", file.c_str(), table_.t.size(),
      table_.t.back(), joints_.size());

    command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      command_topic, rclcpp::QoS(10));
    phase_pub_ = create_publisher<std_msgs::msg::String>("trajectory/phase", rclcpp::QoS(10).transient_local());
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      state_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & msg) {onState(msg);});
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this] {onTimer();});
    setPhase("wait_state");
  }

private:
  // Declares a parameter without a default; throws a clear error when the YAML lacks it.
  rclcpp::Parameter required(const std::string & name, rclcpp::ParameterType type)
  {
    declare_parameter(name, type);
    try {
      rclcpp::Parameter p = get_parameter(name);
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {throw std::runtime_error("not set");}
      return p;
    } catch (const std::exception &) {
      throw std::invalid_argument("parameter '" + name + "' is missing; load config/trajectory.yaml (params_file)");
    }
  }

  void onState(const sensor_msgs::msg::JointState & msg)
  {
    if (phase_ != "wait_state" || msg.position.size() != msg.name.size()) {return;}
    std::map<std::string, double> by_name;
    for (std::size_t k = 0; k < msg.name.size(); ++k) {by_name[msg.name[k]] = msg.position[k];}
    Vec q(static_cast<Eigen::Index>(joints_.size()));
    for (std::size_t j = 0; j < joints_.size(); ++j) {
      const auto it = by_name.find(joints_[j]);
      if (it == by_name.end() || !std::isfinite(it->second)) {return;}
      q[static_cast<Eigen::Index>(j)] = it->second;
    }
    start_ = q;
    approach_s_ = quinticDuration(start_, table_.q.front(), approach_speed_, min_move_s_);
    return_s_ = quinticDuration(table_.q.back(), start_, approach_speed_, min_move_s_);
    phase_start_ = now();
    setPhase("approach");
    RCLCPP_INFO(get_logger(), "start pose read; approach %.2f s", approach_s_);
  }

  void onTimer()
  {
    if (phase_ == "wait_state") {return;}
    double t = (now() - phase_start_).seconds() + lead_s_;
    Vec q;
    if (phase_ == "approach" && t >= approach_s_) {next("track", t, approach_s_);}
    if (phase_ == "track" && t > table_.t.back()) {next(return_to_start_ ? "return" : "hold", t, table_.t.back());}
    if (phase_ == "return" && t >= return_s_) {next("hold", t, return_s_);}
    if (phase_ == "approach") {
      q = quintic(start_, table_.q.front(), approach_s_, t);
    } else if (phase_ == "track") {
      const auto it = std::upper_bound(table_.t.begin(), table_.t.end(), t + 1e-9);
      const std::size_t k = static_cast<std::size_t>(std::distance(table_.t.begin(), it)) - 1;
      q = table_.q[k];
    } else if (phase_ == "return") {
      q = quintic(table_.q.back(), start_, return_s_, t);
    } else {
      q = return_to_start_ ? start_ : table_.q.back();
    }
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joints_;
    msg.position.assign(q.data(), q.data() + q.size());
    command_pub_->publish(msg);
  }

  void next(const std::string & phase, double & t, double duration)
  {
    phase_start_ = phase_start_ + rclcpp::Duration::from_seconds(duration);
    t -= duration;
    setPhase(phase);
  }

  void setPhase(const std::string & phase)
  {
    phase_ = phase;
    std_msgs::msg::String msg;
    msg.data = phase;
    phase_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "phase: %s", phase.c_str());
  }

  std::vector<std::string> joints_;
  Table table_;
  double rate_hz_, approach_speed_, min_move_s_, lead_s_;
  bool return_to_start_;
  std::string phase_;
  rclcpp::Time phase_start_;
  Vec start_;
  double approach_s_ = 0.0, return_s_ = 0.0;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TrajectoryNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
