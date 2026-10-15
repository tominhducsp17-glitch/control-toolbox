// Node "controller": the 1 kHz control loop of one OpenArm arm with the motor bus inside, one thread.
//
//   in : joint_commands (sensor_msgs/JointState)  position = target q (e.g. 100 Hz from the trajectory node);
//                                                  velocity and effort are not used
//   out: joint_states   (sensor_msgs/JointState)  measured q, dq, tau of the arm, state_publish_hz (after the
//                                                  ramp-in)
//   out: controller/reference (sensor_msgs/JointState)  q_d, dq_d (position, velocity) and the torque sent
//                                                  (effort), state_publish_hz
//
// Each cycle (rate_hz, one thread, SCHED_FIFO when allowed):
//   1. take the newest joint_commands (spin_some)
//   2. ControlLoop::step(): read the motors -> safety -> ct::nlc::ReferenceFilter (4th order) -> q_d, dq_d, ddq_d
//      -> ct::nlc::FullComputedTorqueController -> one MIT frame per motor (driver kp = 0)
//   3. publish joint_states and the reference every rate_hz / state_publish_hz cycles
//   4. sleep until the next cycle
// bus: can (openarm_can on can_interface) or sim (SimBus: ct_models OpenArm model with friction).
// Ctrl+C (SIGINT) once: return to the start pose, ramp down, disable; twice: disable at once. SIGTERM (sent by
// ros2 launch after its timeout) only ever requests the return.
// With log_dir set, the run is written to log_dir/<label>/ (log.csv, meta.json) as by ctc_track.
#include "openarm_controller/control_loop.hpp"
#include "openarm_controller/sim_bus.hpp"
#ifdef OPENARM_CONTROLLER_HAVE_CAN
#include "openarm_controller/openarm_can_bus.hpp"
#endif

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <sched.h>
#include <sys/mman.h>

#include <atomic>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using openarm_controller::Vec7;

namespace
{
std::atomic<int> g_stop_requests{0};

void onSigint(int)
{
  g_stop_requests.fetch_add(1);
}

void onSigterm(int)
{
  int expected = 0;
  g_stop_requests.compare_exchange_strong(expected, 1);
}

std::string json(const Vec7 & v)
{
  std::ostringstream out;
  out << '[';
  for (int j = 0; j < v.size(); ++j) {out << (j ? ", " : "") << v[j];}
  out << ']';
  return out.str();
}

const char * phaseName(openarm_controller::Phase phase)
{
  static const char * names[] = {"connect", "ramp", "hold", "track", "return", "ramp down", "stopped", "done"};
  return names[static_cast<int>(phase)];
}
}  // namespace

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("controller")
  {
    // Every parameter comes from the YAML loaded at start (config/controller.yaml); none has a default in the
    // code, so changing a value never needs a rebuild.
    config_.arm = text("arm");
    config_.rate_hz = number("rate_hz");
    config_.filter_wn = number("filter_wn");
    config_.ctc_kp = vec("ctc_kp");
    config_.ctc_kd = vec("ctc_kd");
    if (vec("driver_kp").cwiseAbs().maxCoeff() != 0.0) {
      throw std::invalid_argument("driver_kp must be 0 for every joint (full CTC: the position feedback is on the PC)");
    }
    config_.driver_kd = vec("driver_kd");
    config_.tau_bias = vec("tau_bias");
    config_.friction_fc = vec("friction_fc");
    config_.friction_fv = vec("friction_fv");
    config_.friction_v = number("friction_v");
    config_.ki = vec("ki");
    config_.integral_limit = vec("integral_limit");
    config_.blend_s = number("blend_s");
    config_.hold_kp = vec("hold_kp");
    config_.hold_kd = vec("hold_kd");
    config_.ramp_s = number("ramp_s");
    config_.ramp_down_s = number("ramp_down_s");
    config_.return_speed = number("return_speed");
    config_.min_move_s = number("min_move_s");
    config_.tau_limit = vec("tau_limit");
    config_.max_error = number("max_error");
    config_.max_speed = number("max_speed");
    config_.max_age = number("max_age");
    config_.abort_grace_s = number("abort_grace_s");
    config_.log_max_s = number("log_max_s");
    bus_type_ = text("bus");
    const std::string iface = text("can_interface");
    const Vec7 sim_coulomb = vec("sim_coulomb");
    const Vec7 sim_viscous = vec("sim_viscous");
    const double publish_hz = number("state_publish_hz");
    input_timeout_s_ = number("input_timeout_s");
    log_dir_ = text("log_dir");
    label_ = text("label");
    for (int j = 1; j <= openarm_controller::kJoints; ++j) {
      joints_.push_back("openarm_" + config_.arm + "_joint" + std::to_string(j));
    }

    if (bus_type_ == "sim") {
      openarm_controller::SimBusOptions options;
      options.arm = config_.arm;
      options.cycle_s = 1.0 / config_.rate_hz;
      options.coulomb_nm = sim_coulomb;
      options.viscous_nms = sim_viscous;
      bus_ = std::make_unique<openarm_controller::SimBus>(options);
    } else if (bus_type_ == "can") {
#ifdef OPENARM_CONTROLLER_HAVE_CAN
      bus_ = std::make_unique<openarm_controller::OpenArmCanBus>(iface);
#else
      throw std::invalid_argument("bus 'can': built without openarm_can (install it and rebuild)");
#endif
    } else {
      throw std::invalid_argument("bus must be can or sim");
    }
    loop_ = std::make_unique<openarm_controller::ControlLoop>(*bus_, config_);
    publish_every_ = std::max(1, static_cast<int>(std::lround(config_.rate_hz / publish_hz)));

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>(text("state_topic"), rclcpp::QoS(10));
    reference_pub_ = create_publisher<sensor_msgs::msg::JointState>("controller/reference", rclcpp::QoS(10));
    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      text("command_topic"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & m) {onCommand(m);});
  }

  // The control loop until the motors are disabled; returns the process exit code.
  int run()
  {
    sched_param sp{};
    sp.sched_priority = 80;
    fifo_ = bus_->realtime() && sched_setscheduler(0, SCHED_FIFO, &sp) == 0;
    if (bus_->realtime()) {mlockall(MCL_CURRENT | MCL_FUTURE);}
    RCLCPP_INFO(get_logger(), "arm %s, bus %s, %.0f Hz%s, reference filter wn %.0f rad/s, full CTC (ct_nlc)",
      config_.arm.c_str(), bus_type_.c_str(), config_.rate_hz, fifo_ ? " (SCHED_FIFO)" : "", config_.filter_wn);
    if (!loop_->start()) {
      RCLCPP_ERROR(get_logger(), "%s", loop_->reason().c_str());
      return 2;
    }
    RCLCPP_INFO(get_logger(), "motors enabled; following %s (Ctrl+C: return to the start pose and disable)",
      command_sub_->get_topic_name());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(shared_from_this());
    int handled_stops = 0;
    long cycle = 0;
    openarm_controller::Phase last_phase = loop_->phase();
    bool stale_input = false;
    for (;;) {
      executor.spin_some(std::chrono::nanoseconds(0));
      for (; handled_stops < g_stop_requests.load(); ++handled_stops) {loop_->requestStop();}
      if (!loop_->step()) {break;}
      if (loop_->phase() != last_phase) {
        last_phase = loop_->phase();
        RCLCPP_INFO(get_logger(), "phase: %s %s", phaseName(last_phase), loop_->reason().c_str());
      }
      const bool stale = loop_->phase() == openarm_controller::Phase::kTrack &&
        loop_->secondsSinceTarget() > input_timeout_s_;
      if (stale && !stale_input) {
        RCLCPP_WARN(get_logger(), "no joint_commands for %.3f s: holding the last target", input_timeout_s_);
      }
      stale_input = stale;
      // joint_states only once the gains are ramped in: the trajectory node starts its approach from the first
      // joint_states, so its first targets are the current pose and the reference filter starts at rest
      if (++cycle % publish_every_ == 0 && loop_->phase() != openarm_controller::Phase::kRamp) {publish();}
      loop_->waitNextCycle();
    }
    RCLCPP_INFO(get_logger(), "%s %s; overruns %d, clamped cycles %d", loop_->status().c_str(),
      loop_->reason().c_str(), loop_->overruns(), loop_->clampedCycles());
    writeRun();
    return loop_->status() == "completed" ? 0 : 1;
  }

private:
  // Declares a parameter without a default; throws a clear error when the YAML lacks it.
  rclcpp::Parameter required(const std::string & name, rclcpp::ParameterType type)
  {
    declare_parameter(name, type);
    rclcpp::Parameter p = get_parameter(name);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      throw std::invalid_argument("parameter '" + name + "' is missing; load config/controller.yaml (params_file)");
    }
    return p;
  }
  std::string text(const std::string & name)
  {
    return required(name, rclcpp::ParameterType::PARAMETER_STRING).as_string();
  }
  double number(const std::string & name)
  {
    return required(name, rclcpp::ParameterType::PARAMETER_DOUBLE).as_double();
  }
  Vec7 vec(const std::string & name)
  {
    const auto v = required(name, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY).as_double_array();
    if (v.size() != openarm_controller::kJoints) {throw std::invalid_argument(name + " needs 7 values (J1..J7)");}
    return Eigen::Map<const Vec7>(v.data());
  }

  void onCommand(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size()) {return;}
    std::map<std::string, double> by_name;
    for (std::size_t k = 0; k < msg.name.size(); ++k) {by_name[msg.name[k]] = msg.position[k];}
    Vec7 q;
    for (int j = 0; j < openarm_controller::kJoints; ++j) {
      const auto it = by_name.find(joints_[static_cast<std::size_t>(j)]);
      if (it == by_name.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_commands without %s: ignored",
          joints_[static_cast<std::size_t>(j)].c_str());
        return;
      }
      q[j] = it->second;
    }
    loop_->setTarget(q);
  }

  void publish()
  {
    const auto & s = loop_->state();
    const auto & r = loop_->lastRow();
    sensor_msgs::msg::JointState state, reference;
    state.header.stamp = reference.header.stamp = now();
    state.name = reference.name = joints_;
    state.position.assign(s.q.data(), s.q.data() + openarm_controller::kJoints);
    state.velocity.assign(s.dq.data(), s.dq.data() + openarm_controller::kJoints);
    state.effort.assign(s.tau.data(), s.tau.data() + openarm_controller::kJoints);
    reference.position.assign(r.q_d.data(), r.q_d.data() + openarm_controller::kJoints);
    reference.velocity.assign(r.dq_d.data(), r.dq_d.data() + openarm_controller::kJoints);
    reference.effort.assign(r.tau_sent.data(), r.tau_sent.data() + openarm_controller::kJoints);
    state_pub_->publish(state);
    reference_pub_->publish(reference);
  }

  void writeRun()
  {
    if (log_dir_.empty() || loop_->log().empty()) {return;}
    std::string label = label_;
    if (label.empty()) {
      char stamp[32];
      const std::time_t now_t = std::time(nullptr);
      std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&now_t));
      label = "controller-" + bus_type_ + "-" + stamp;
    }
    const std::filesystem::path dir = std::filesystem::path(log_dir_) / label;
    std::filesystem::create_directories(dir);
    loop_->writeLog((dir / "log.csv").string());
    std::ofstream meta(dir / "meta.json");
    meta << "{\n  \"label\": \"" << label << "\", \"node\": \"openarm_controller/controller_node\",\n"
         << "  \"mode\": \"" << (bus_type_ == "sim" ? "sim" : "hardware") << "\", \"arm\": \"" << config_.arm
         << "\", \"rate_hz\": " << config_.rate_hz << ", \"sched_fifo\": " << (fifo_ ? "true" : "false") << ",\n"
         << "  \"filter_wn\": " << config_.filter_wn << ",\n"
         << "  \"ctc_kp\": " << json(config_.ctc_kp) << ", \"ctc_kd\": " << json(config_.ctc_kd)
         << ", \"driver_kp\": [0, 0, 0, 0, 0, 0, 0], \"driver_kd\": " << json(config_.driver_kd) << ",\n"
         << "  \"tau_bias\": " << json(config_.tau_bias) << ", \"friction_fc\": " << json(config_.friction_fc)
         << ", \"friction_fv\": " << json(config_.friction_fv) << ", \"friction_v\": " << config_.friction_v << ",\n"
         << "  \"ki\": " << json(config_.ki) << ", \"integral_limit\": " << json(config_.integral_limit) << ",\n"
         << "  \"tau_limit\": " << json(config_.tau_limit) << ", \"max_error\": " << config_.max_error
         << ", \"max_speed\": " << config_.max_speed << ", \"max_age\": " << config_.max_age << ",\n"
         << "  \"status\": \"" << loop_->status() << "\", \"reason\": \"" << loop_->reason() << "\",\n"
         << "  \"overruns\": " << loop_->overruns() << ", \"clamped_cycles\": " << loop_->clampedCycles()
         << ", \"rows\": " << loop_->log().size() << "\n}\n";
    RCLCPP_INFO(get_logger(), "-> %s", dir.c_str());
  }

  openarm_controller::LoopConfig config_;
  std::string bus_type_, log_dir_, label_;
  std::vector<std::string> joints_;
  double input_timeout_s_ = 0.0;
  int publish_every_ = 1;
  bool fifo_ = false;
  std::unique_ptr<openarm_controller::ArmBus> bus_;
  std::unique_ptr<openarm_controller::ControlLoop> loop_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_, reference_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
};

int main(int argc, char ** argv)
{
  // own signal handling: Ctrl+C must not end the loop, only request the return (once) or the disable (twice)
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSigint);
  std::signal(SIGTERM, onSigterm);
  int code = 0;
  try {
    auto node = std::make_shared<ControllerNode>();
    code = node->run();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("controller"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
