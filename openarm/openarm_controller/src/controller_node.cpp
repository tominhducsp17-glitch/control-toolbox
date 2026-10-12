// Node 2 "controller": calls a ct_nlc controller (ct::core::Controller with the generated OpenArm
// model of ct_models) and outputs tau_ff.
//
//   in : joint_states   (sensor_msgs/JointState)  measured position/velocity
//   in : joint_commands (sensor_msgs/JointState)  position = q_target, velocity = dq_target
//   out: controller/tau_ff (sensor_msgs/JointState) effort = tau_ff [N m] for the joints of the command,
//        stamped with the command's stamp
//
// Each received command (event driven):
//   1. reference q_d, dq_d and ddq_d (estimated from dq_d): controller->setReference()
//   2. state x = [q; dq] measured: controller->computeControl(x, t, u)   (override in each controller,
//      which evaluates M, C, G with its model; chosen by controller_type)
// Joints of joint_names that a command does not contain (e.g. the other arm) are held at their
// measured position with zero velocity and get no output.
#include <ct/models/OpenArm/OpenArm.h>
#include <ct/nlc/nlc.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

template<size_t NJ>
using JointVector = Eigen::Matrix<double, NJ, 1>;
template<size_t NJ>
using Model = ct::models::OpenArm::OpenArmModel<NJ>;
template<size_t NJ>
using Controller = ct::nlc::ModelBasedJointController<NJ, Model<NJ>>;

// controller_type -> controller. Register new ct_nlc controllers here.
template<size_t NJ>
std::unique_ptr<Controller<NJ>> makeController(const std::string & type, const Model<NJ> & model)
{
  if (type == "ctc_feedforward") {  // M(q_d) ddq_d + C(q_d, dq_d) dq_d + G(q_d)
    return std::make_unique<ct::nlc::ComputedTorqueController<NJ, Model<NJ>>>(model);
  }
  if (type == "gravity_compensation") {  // G(q_d)
    return std::make_unique<ct::nlc::GravityCompensationController<NJ, Model<NJ>>>(model);
  }
  throw std::invalid_argument("unknown controller_type '" + type + "' (ctc_feedforward | gravity_compensation)");
}

// Declares a parameter without a default and returns it; throws a clear error when the YAML lacks it.
rclcpp::Parameter required(rclcpp::Node & node, const std::string & name, rclcpp::ParameterType type)
{
  node.declare_parameter(name, type);
  try {
    rclcpp::Parameter p = node.get_parameter(name);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {throw std::runtime_error("not set");}
    return p;
  } catch (const std::exception &) {
    throw std::invalid_argument("parameter '" + name + "' is missing; load config/controller.yaml (params_file)");
  }
}

template<size_t NJ>
class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const std::vector<std::string> & joints)
  : Node("controller"), joints_(joints)
  {
    // Every parameter comes from the YAML loaded at start (config/controller.yaml); none has a
    // default in the code, so changing a value never needs a rebuild.
    required(*this, "joint_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
    estimator_ = std::make_unique<ct::nlc::ReferenceAccelerationEstimator<NJ>>(
      required(*this, "acceleration_cutoff_hz", rclcpp::ParameterType::PARAMETER_DOUBLE).as_double(),
      required(*this, "max_abs_acceleration", rclcpp::ParameterType::PARAMETER_DOUBLE).as_double());
    for (const auto & [name, type] : std::vector<std::pair<std::string, rclcpp::ParameterType>>{
        {"controller_type", rclcpp::ParameterType::PARAMETER_STRING},
        {"torque_limit", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY},
        {"input_timeout_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"state_topic", rclcpp::ParameterType::PARAMETER_STRING},
        {"command_topic", rclcpp::ParameterType::PARAMETER_STRING},
        {"output_topic", rclcpp::ParameterType::PARAMETER_STRING}})
    {
      required(*this, name, type);
    }
    type_ = get_parameter("controller_type").as_string();
    controller_ = makeController<NJ>(type_, Model<NJ>(joints_));
    const auto limit = get_parameter("torque_limit").as_double_array();
    if (limit.size() != NJ) {throw std::invalid_argument("torque_limit must have one value per joint");}
    controller_->setTorqueLimit(Eigen::Map<const JointVector<NJ>>(limit.data()));
    timeout_s_ = get_parameter("input_timeout_s").as_double();
    for (std::size_t j = 0; j < NJ; ++j) {index_[joints_[j]] = j;}
    q_meas_.setZero();
    dq_meas_.setZero();

    tau_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      get_parameter("output_topic").as_string(), rclcpp::QoS(10));
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("state_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & m) {onState(m);});
    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("command_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & m) {onCommand(m);});
    watchdog_ = create_wall_timer(std::chrono::milliseconds(10), [this] {onWatchdog();});
    RCLCPP_INFO(get_logger(), "controller '%s' (ct_nlc), %zu joints, model: ct_models OpenArm (generated code)",
      type_.c_str(), NJ);
  }

private:
  void onState(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size()) {return;}
    for (std::size_t k = 0; k < msg.name.size(); ++k) {
      const auto it = index_.find(msg.name[k]);
      if (it == index_.end()) {continue;}  // fingers: fixed at 0 in the generated model
      if (!std::isfinite(msg.position[k])) {continue;}
      q_meas_[it->second] = msg.position[k];
      dq_meas_[it->second] = msg.velocity.size() == msg.name.size() ? msg.velocity[k] : 0.0;
      measured_[it->second] = true;
    }
  }

  void onCommand(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_commands without positions");
      return;
    }
    const bool has_velocity = msg.velocity.size() == msg.name.size();
    if (!has_velocity) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_commands without velocity; dq_target = 0");
    }
    JointVector<NJ> q_d = q_meas_;  // joints without a command hold their measured pose
    JointVector<NJ> dq_d = JointVector<NJ>::Zero();
    std::vector<std::size_t> commanded;
    for (std::size_t k = 0; k < msg.name.size(); ++k) {
      const auto it = index_.find(msg.name[k]);
      if (it == index_.end()) {continue;}
      q_d[it->second] = msg.position[k];
      dq_d[it->second] = has_velocity ? msg.velocity[k] : 0.0;
      commanded.push_back(it->second);
    }
    if (commanded.empty()) {return;}
    for (std::size_t j = 0; j < NJ; ++j) {
      const bool in_command = std::find(commanded.begin(), commanded.end(), j) != commanded.end();
      if (!in_command && !measured_[j]) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "%s has neither a command nor a measurement yet; tau_ff not computed", joints_[j].c_str());
        return;
      }
    }
    const bool stamped = msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0;
    const double t = stamped ? rclcpp::Time(msg.header.stamp).seconds() : now().seconds();
    try {
      controller_->setReference(q_d, dq_d, estimator_->update(t, dq_d));
      ct::core::StateVector<2 * NJ> x;
      x << q_meas_, dq_meas_;
      ct::core::ControlVector<NJ> u;
      controller_->computeControl(x, t, u);  // override of ct::core::Controller
      sensor_msgs::msg::JointState out;
      out.header.stamp = stamped ? msg.header.stamp : builtin_interfaces::msg::Time(now());
      for (const auto j : commanded) {
        out.name.push_back(joints_[j]);
        out.effort.push_back(u[j]);
      }
      tau_pub_->publish(out);
      last_command_ = now();
      timed_out_ = false;
      if (controller_->lastControlSaturated()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tau_ff clamped by torque_limit");
      }
    } catch (const std::invalid_argument & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "command rejected: %s", e.what());
    }
  }

  void onWatchdog()
  {
    if (!last_command_ || timed_out_ || (now() - *last_command_).seconds() <= timeout_s_) {return;}
    timed_out_ = true;
    estimator_->reset();
    RCLCPP_WARN(get_logger(), "no joint_commands for %.3f s", timeout_s_);
  }

  std::vector<std::string> joints_;
  std::map<std::string, std::size_t> index_;
  std::string type_;
  std::unique_ptr<Controller<NJ>> controller_;
  std::unique_ptr<ct::nlc::ReferenceAccelerationEstimator<NJ>> estimator_;
  JointVector<NJ> q_meas_, dq_meas_;
  std::array<bool, NJ> measured_{};
  double timeout_s_;
  std::optional<rclcpp::Time> last_command_;
  bool timed_out_ = false;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tau_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_, command_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

std::vector<std::string> jointNamesParameter()
{
  // Short-lived node with the same name, so the YAML for "controller" applies to it.
  auto probe = std::make_shared<rclcpp::Node>("controller");
  return required(*probe, "joint_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY).as_string_array();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    const auto joints = jointNamesParameter();
    std::shared_ptr<rclcpp::Node> node;
    if (joints.size() == 7) {
      node = std::make_shared<ControllerNode<7>>(joints);
    } else if (joints.size() == 14) {
      node = std::make_shared<ControllerNode<14>>(joints);
    } else {
      throw std::invalid_argument("joint_names must list 7 (one arm) or 14 (both arms) joints");
    }
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("controller"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
