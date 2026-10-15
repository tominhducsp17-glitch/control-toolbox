// Motor bus of one OpenArm arm (7 joints, Damiao MIT mode). Two implementations: OpenArmCanBus (the real arm
// through openarm_can) and SimBus (rigid-body plant with the ct_models OpenArm model, for tests).
#pragma once

#include <Eigen/Core>

#include <array>

namespace openarm_controller
{

constexpr int kJoints = 7;
using Vec7 = Eigen::Matrix<double, kJoints, 1>;

// Damiao status nibble: codes >= 8 are faults (openarm_can MOTOR_ERROR_THRESHOLD).
constexpr int kMotorFaultThreshold = 8;

// One MIT frame per motor; the driver applies tau_motor = kp (q - q_m) + kd (dq - dq_m) + tau.
struct MotorCommand
{
  Vec7 q = Vec7::Zero();
  Vec7 dq = Vec7::Zero();
  Vec7 kp = Vec7::Zero();
  Vec7 kd = Vec7::Zero();
  Vec7 tau = Vec7::Zero();
};

struct ArmState
{
  Vec7 q = Vec7::Zero();
  Vec7 dq = Vec7::Zero();
  Vec7 tau = Vec7::Zero();
  std::array<double, kJoints> age_s{};  // seconds since each motor's last reply
  std::array<int, kJoints> error{};     // Damiao status nibble
  std::array<bool, kJoints> enabled{};
};

class ArmBus
{
public:
  virtual ~ArmBus();
  virtual void enable() = 0;
  virtual void disable() = 0;
  // State query only (no torque); the replies are read before returning.
  virtual void refresh() = 0;
  // One MIT frame per motor; the replies are read before returning.
  virtual void command(const MotorCommand & command) = 0;
  virtual ArmState read() = 0;
  // Seconds of the bus clock (steady clock on hardware, simulated time in SimBus).
  virtual double now() const = 0;
  // True when the caller must pace the loop in wall-clock time (hardware).
  virtual bool realtime() const = 0;
};

}  // namespace openarm_controller
