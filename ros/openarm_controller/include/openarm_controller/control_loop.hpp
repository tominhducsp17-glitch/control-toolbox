// The 1 kHz control loop of one arm, one thread: read the motors -> safety -> reference filter -> full computed-
// torque control (ct_nlc) -> one MIT frame per motor. Independent of ROS: controller_node feeds it the target
// positions (joint_commands, ~100 Hz) and calls step() once per cycle; the tests drive it with SimBus.
//
// Phases (numbers as in the CSV log):
//   0 connect   read only (connect_s), check every motor replies, has no fault, is disabled and inside the limits
//   1 ramp      motors enabled, driver PD (hold_kp, hold_kd) at the start pose + model feedforward, gains 0 -> 1
//   2 hold      driver PD at the start pose until the first target arrives
//   3 track     target -> ReferenceFilter -> q_d, dq_d, ddq_d; FullComputedTorqueController, blended in from the
//               driver PD over blend_s (driver kp = 0, kd = driver_kd); the integral runs in this phase only
//   6 stopped   after a soft stop (tracking error, speed, position limits): the filter brings the reference to
//               rest, full CTC blends out to the driver PD, new targets are ignored
//   4 return    after a user stop (Ctrl+C once): quintic from the current reference back to the start pose
//   5 ramp down gains and torque 1 -> 0, then disable
//   7 done
// Hard stops disable at once: stale feedback, motor fault, a second user stop, or a soft-stop condition during the
// stop/return outside abort_grace_s.
#pragma once

#include "openarm_controller/arm_bus.hpp"

#include <ct/models/OpenArm/OpenArm.h>
#include <ct/nlc/nlc.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace openarm_controller
{

enum class Phase : int
{
  kConnect = 0, kRamp = 1, kHold = 2, kTrack = 3, kReturn = 4, kRampDown = 5, kStopped = 6, kDone = 7,
};

struct LoopConfig
{
  std::string arm = "right";
  double rate_hz = 1000.0;
  // reference: 4th-order filter of the target positions (ct::nlc::ReferenceFilter)
  double filter_wn = 60.0;          // [rad/s]
  // full CTC (ct::nlc::FullComputedTorqueController), torque units
  Vec7 ctc_kp = Vec7::Zero();       // [N m/rad]
  Vec7 ctc_kd = Vec7::Zero();       // [N m s/rad]
  Vec7 driver_kd = Vec7::Zero();    // damping left in the motor driver under full CTC (0 = pure torque command)
  Vec7 tau_bias = Vec7::Zero();     // measured model correction [N m]
  Vec7 friction_fc = Vec7::Zero();  // [N m]
  Vec7 friction_fv = Vec7::Zero();  // [N m s/rad]
  double friction_v = 0.02;         // [rad/s]
  Vec7 ki = Vec7::Zero();           // [N m/(rad s)]
  Vec7 integral_limit = Vec7::Zero();  // [N m]
  double blend_s = 0.5;             // driver PD <-> full CTC crossfade [s]
  // driver PD for ramp, hold, return (+ model feedforward)
  Vec7 hold_kp = Vec7::Zero();      // [N m/rad], 0..500
  Vec7 hold_kd = Vec7::Zero();      // [N m s/rad], 0..5
  double connect_s = 0.2;
  double ramp_s = 1.0;
  double ramp_down_s = 1.0;
  double return_speed = 0.25;       // peak speed of the return quintic [rad/s]
  double min_move_s = 2.0;
  // safety
  Vec7 tau_limit = Vec7::Constant(1.0);  // clamp of the torque sent [N m]
  double max_error = 0.15;          // soft stop: |q - q_d| [rad]
  double max_speed = 5.0;           // soft stop: |dq| [rad/s]
  double max_age = 0.006;           // hard stop: feedback older than this [s]
  double abort_grace_s = 0.3;       // after a soft stop, seconds before speed/error become hard stops
  double position_margin = 0.2;     // soft stop outside the URDF limits + margin [rad]
  double log_max_s = 300.0;         // rows reserved for the log (no allocation in the loop); 0: no log
};

struct LogRow
{
  double t = 0.0;
  int phase = 0;
  double scale = 0.0;
  double track_t = -1.0;            // time in the track phase, else -1
  Vec7 q_d, dq_d, ddq_d, q, dq, tau_measured, tau_sent, target;
  double max_age = 0.0;
  double loop_us = 0.0;
  bool clamped = false;
  double blend = 0.0;
};

class ControlLoop
{
public:
  ControlLoop(ArmBus & bus, const LoopConfig & config);
  virtual ~ControlLoop();

  // Phase 0 (connect, blocking for connect_s), then enables the motors. False with reason() when a check failed.
  bool start();
  // New target position (from the trajectory source). Ignored after a stop.
  void setTarget(const Vec7 & q);
  // Ctrl+C: the first request returns to the start pose and disables, the second disables at once.
  void requestStop();
  // One control cycle. False when the loop has finished (motors disabled).
  bool step();
  // Sleeps until the next cycle in wall-clock time (counts overruns); the node calls it, the tests do not.
  void waitNextCycle();

  Phase phase() const;
  const std::string & status() const;  // running | completed | stopped | hard_stop | not_started
  const std::string & reason() const;
  const ArmState & state() const;
  const LogRow & lastRow() const;
  const std::vector<LogRow> & log() const;
  int overruns() const;
  int clampedCycles() const;
  double secondsSinceTarget() const;   // +inf before the first target
  // CSV with the columns of ctc_track (tools/plot_*.py of openarm_control read it)
  void writeLog(const std::string & path) const;

private:
  struct Reference
  {
    Vec7 q = Vec7::Zero(), dq = Vec7::Zero(), ddq = Vec7::Zero();
  };
  double elapsed() const;
  void enterPhase(Phase phase, double t);
  void softStop(const std::string & why, double t);
  void hardStop(const std::string & why, double t);
  Reference reference(double t, double tp, double & scale);
  double blend(double t) const;
  bool checkSafety(double t);

  typedef ct::models::OpenArm::OpenArmModel<kJoints> Model;
  ArmBus & bus_;
  LoopConfig config_;
  double period_;
  Vec7 q_lower_, q_upper_;
  std::unique_ptr<ct::nlc::ComputedTorqueController<kJoints, Model>> feedforward_;
  std::unique_ptr<ct::nlc::FullComputedTorqueController<kJoints, Model>> full_ctc_;
  ct::nlc::ReferenceFilter<kJoints> filter_;

  Phase phase_ = Phase::kConnect;
  std::string status_ = "not_started";
  std::string reason_;
  double t0_ = 0.0;
  double phase_start_ = 0.0;
  ArmState state_;
  Vec7 q0_ = Vec7::Zero();
  Vec7 target_ = Vec7::Zero();
  bool has_target_ = false;
  double target_t_ = -1.0;
  int stop_requests_ = 0;
  Reference last_ref_;
  Reference return_from_;
  double return_s_ = 0.0;
  Vec7 stop_target_ = Vec7::Zero();
  double blend_from_ = 0.0;         // blend at the start of the stop/return (fades out from there)
  double stop_t_ = -1.0;
  Vec7 prev_q_d_ = Vec7::Zero();
  std::vector<LogRow> log_;
  LogRow row_;
  int overruns_ = 0;
  int clamped_cycles_ = 0;
  std::chrono::steady_clock::time_point next_tick_;
  bool ticking_ = false;
};

}  // namespace openarm_controller
