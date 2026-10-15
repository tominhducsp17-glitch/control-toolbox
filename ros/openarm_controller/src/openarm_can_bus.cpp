#include "openarm_controller/openarm_can_bus.hpp"

#include <openarm/can/socket/openarm.hpp>

#include <chrono>
#include <limits>
#include <vector>

namespace openarm_controller
{

namespace dm = openarm::damiao_motor;

struct OpenArmCanBus::Impl
{
  Impl(const std::string & iface, int timeout)
  : arm(iface, true), recv_timeout_us(timeout) {}
  openarm::can::socket::OpenArm arm;
  int recv_timeout_us;
  bool enabled = false;
};

OpenArmCanBus::OpenArmCanBus(const std::string & can_interface, int recv_timeout_us)
: impl_(std::make_unique<Impl>(can_interface, recv_timeout_us))
{
  // OpenArm v1: J1-J2 DM8009, J3-J4 DM4340, J5-J7 DM4310; send 0x01-0x07, reply 0x11-0x17.
  impl_->arm.init_arm_motors(
    {dm::MotorType::DM8009, dm::MotorType::DM8009, dm::MotorType::DM4340, dm::MotorType::DM4340,
      dm::MotorType::DM4310, dm::MotorType::DM4310, dm::MotorType::DM4310},
    {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
    {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17});
  impl_->arm.set_callback_mode_all(dm::CallbackMode::STATE);
  impl_->arm.recv_all(2000);  // drain
}

OpenArmCanBus::~OpenArmCanBus()
{
  // Last line of defence: an exception between enable() and disable() still disables the motors.
  if (impl_ && impl_->enabled) {
    try {
      disable();
    } catch (...) {
    }
  }
}

void OpenArmCanBus::enable()
{
  impl_->enabled = true;
  impl_->arm.get_arm().enable_all();
  impl_->arm.recv_all(impl_->recv_timeout_us);
}

void OpenArmCanBus::disable()
{
  impl_->arm.get_arm().disable_all();
  impl_->arm.recv_all(impl_->recv_timeout_us);
  impl_->enabled = false;
}

void OpenArmCanBus::refresh()
{
  impl_->arm.get_arm().refresh_all();
  impl_->arm.recv_all(impl_->recv_timeout_us);
}

void OpenArmCanBus::command(const MotorCommand & command)
{
  std::vector<dm::MITParam> params(kJoints);
  for (int j = 0; j < kJoints; ++j) {
    params[j] = dm::MITParam{command.kp[j], command.kd[j], command.q[j], command.dq[j], command.tau[j]};
  }
  impl_->arm.get_arm().mit_control_all(params);
  impl_->arm.recv_all(impl_->recv_timeout_us);
}

ArmState OpenArmCanBus::read()
{
  ArmState state;
  auto & arm = impl_->arm.get_arm();
  const auto motors = arm.get_motors();
  for (int j = 0; j < kJoints; ++j) {
    const auto & m = motors[static_cast<std::size_t>(j)];
    state.q[j] = m.get_position();
    state.dq[j] = m.get_velocity();
    state.tau[j] = m.get_torque();
    state.error[j] = m.get_error_code();
    state.enabled[j] = m.is_enabled();
    const auto & stats = arm.get_link_stats(j);
    state.age_s[j] = stats.ever_responded() ?
      1e-6 * static_cast<double>(stats.since_last_response().count()) :
      std::numeric_limits<double>::infinity();
  }
  return state;
}

bool OpenArmCanBus::realtime() const
{
  return true;
}

double OpenArmCanBus::now() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace openarm_controller
