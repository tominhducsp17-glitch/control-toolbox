// ControlLoop on the simulated arm (SimBus, joint friction measured on the robot): tracking of a 100 Hz position
// stream at 1 kHz, the user stop (return, ramp down, disable), soft stops and hard stops.
#include "openarm_controller/control_loop.hpp"
#include "openarm_controller/sim_bus.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace openarm_controller;

namespace
{
const Vec7 kHang = (Vec7() << 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0).finished();
const Vec7 kGoal = (Vec7() << 0.4, 0.25, 0.0, 1.4, 0.0, 0.0, 0.0).finished();

SimBusOptions simArm()
{
  SimBusOptions o;
  o.q0 = kHang;
  o.cycle_s = 0.001;
  o.coulomb_nm << 0.57, 0.59, 0.49, 1.04, 0.2, 0.09, 0.12;
  o.viscous_nms << 0.59, 0.53, 0.13, 1.94, 0.2, 0.02, 0.19;
  return o;
}

// config/controller.yaml
LoopConfig tunedConfig()
{
  LoopConfig c;
  c.rate_hz = 1000.0;
  c.filter_wn = 60.0;
  c.ctc_kp << 100, 70, 60, 60, 15, 22, 25;
  c.ctc_kd << 5, 7, 4.8, 4.8, 0.18, 0.42, 0.5;
  c.friction_fc << 0.55, 0.56, 0.46, 1.05, 0.15, 0.07, 0.10;
  c.friction_fv << 0, 0, 0, 0.3, 0, 0, 0;
  c.ki << 40, 30, 30, 40, 0, 0, 0;
  c.integral_limit << 1.0, 0.5, 0.5, 1.0, 0, 0, 0;
  c.hold_kp << 70, 70, 70, 60, 35, 35, 35;
  c.hold_kd << 2.75, 2.5, 2.0, 2.0, 1.2, 1.2, 1.2;
  c.ramp_down_s = 0.5;
  c.tau_limit << 40, 40, 27, 27, 7, 7, 7;
  c.max_speed = 5.445;
  c.log_max_s = 60.0;
  return c;
}

// Target positions as the trajectory node sends them: 100 Hz, plan at t + lead. Quintic hang -> goal in 3 s,
// starting at 0.5 s, then hold.
Vec7 plan(double t)
{
  const double s = std::clamp((t - 0.5) / 3.0, 0.0, 1.0);
  return kHang + s * s * s * (10 - 15 * s + 6 * s * s) * (kGoal - kHang);
}

struct Result
{
  double max_track_error = 0.0;   // |q_d - q| in the track phase (after the blend)
  double max_to_plan = 0.0;       // |q_d - plan(t)|: the filtered reference against the intended path
  double max_q_step = 0.0;        // largest change of the command q_d between cycles
  double max_dq_step = 0.0;
};

// Runs the loop; the 100 Hz stream starts with the track phase. stop_at: seconds into the stream for the user
// stop(s).
Result run(ControlLoop & loop, double stream_s, double stop_at = -1.0, int stops = 1)
{
  Result r;
  EXPECT_TRUE(loop.start()) << loop.reason();
  const double lead = 4.0 / 60.0 + 0.005;
  double stream_t0 = -1.0;
  int cycle = 0;
  bool stopped = false;
  Vec7 prev_q_d = Vec7::Zero(), prev_dq_d = Vec7::Zero();
  while (cycle < 60000) {
    const double t = cycle * 0.001;
    if (loop.phase() != Phase::kRamp && stream_t0 < 0.0) {stream_t0 = t;}
    if (stream_t0 >= 0.0 && cycle % 10 == 0 && t - stream_t0 <= stream_s) {
      const double ts = std::floor((t - stream_t0 + lead) * 100.0) / 100.0;
      loop.setTarget(plan(ts));
    }
    if (stop_at >= 0.0 && !stopped && stream_t0 >= 0.0 && t - stream_t0 >= stop_at) {
      for (int k = 0; k < stops; ++k) {loop.requestStop();}
      stopped = true;
    }
    if (!loop.step()) {break;}
    const LogRow & row = loop.lastRow();
    if (cycle > 0) {
      r.max_q_step = std::max(r.max_q_step, (row.q_d - prev_q_d).cwiseAbs().maxCoeff());
      r.max_dq_step = std::max(r.max_dq_step, (row.dq_d - prev_dq_d).cwiseAbs().maxCoeff());
    }
    prev_q_d = row.q_d;
    prev_dq_d = row.dq_d;
    if (row.phase == static_cast<int>(Phase::kTrack) && row.blend >= 1.0) {
      r.max_track_error = std::max(r.max_track_error, (row.q_d - row.q).cwiseAbs().maxCoeff());
      r.max_to_plan = std::max(r.max_to_plan, (row.q_d - plan(row.track_t)).cwiseAbs().maxCoeff());
    }
    ++cycle;
  }
  return r;
}
}  // namespace

TEST(ControlLoop, TracksA100HzStreamAndReturnsOnUserStop)
{
  SimBus bus(simArm());
  ControlLoop loop(bus, tunedConfig());
  const Result r = run(loop, 6.0, 6.0);
  EXPECT_EQ(loop.status(), "completed") << loop.reason();
  EXPECT_FALSE(bus.enabled());
  EXPECT_EQ(bus.enableCalls(), 1);
  // J4 at 0.84 rad/s: the simulated viscous friction (1.94) exceeds the feedforward (0.3) by ~1.4 N m
  EXPECT_LT(r.max_track_error, 0.02);
  EXPECT_LT(r.max_to_plan, 0.003);     // filter + look-ahead: the reference stays on the plan
  EXPECT_LT(r.max_q_step, 2e-3);       // no step in the command (100 Hz stream, phase changes, return)
  EXPECT_LT(r.max_dq_step, 0.05);
  EXPECT_LT((bus.trueQ() - kHang).cwiseAbs().maxCoeff(), 0.03);  // back at the start pose
  EXPECT_EQ(loop.clampedCycles(), 0);
  bool saw_full_ctc = false;
  int ramp_down_rows = 0;
  for (const auto & row : loop.log()) {
    saw_full_ctc |= row.blend >= 1.0;
    ramp_down_rows += row.phase == static_cast<int>(Phase::kRampDown) ? 1 : 0;
  }
  EXPECT_TRUE(saw_full_ctc);
  EXPECT_NEAR(ramp_down_rows, 500, 2);  // the torque fades over ramp_down_s before the disable
}

TEST(ControlLoop, SecondUserStopDisablesAtOnce)
{
  SimBus bus(simArm());
  ControlLoop loop(bus, tunedConfig());
  run(loop, 6.0, 2.0, 2);
  EXPECT_EQ(loop.status(), "hard_stop");
  EXPECT_NE(loop.reason().find("second user stop"), std::string::npos) << loop.reason();
  EXPECT_FALSE(bus.enabled());
}

TEST(ControlLoop, TrackingErrorSoftStopHoldsWithoutStepThenReturns)
{
  SimBus bus(simArm());
  LoopConfig c = tunedConfig();
  c.max_error = 0.004;  // reached while the arm accelerates
  ControlLoop loop(bus, c);
  const Result r = run(loop, 6.0, 5.0);
  EXPECT_EQ(loop.status(), "stopped");
  EXPECT_NE(loop.reason().find("tracking error"), std::string::npos) << loop.reason();
  EXPECT_FALSE(bus.enabled());
  EXPECT_LT(r.max_q_step, 2e-3);
  bool saw_stopped = false;
  for (const auto & row : loop.log()) {saw_stopped |= row.phase == static_cast<int>(Phase::kStopped);}
  EXPECT_TRUE(saw_stopped);
  EXPECT_LT((bus.trueQ() - kHang).cwiseAbs().maxCoeff(), 0.03);
}

TEST(ControlLoop, StaleFeedbackIsHardStop)
{
  SimBusOptions o = simArm();
  o.stale_after_s = 2.0;
  SimBus bus(o);
  ControlLoop loop(bus, tunedConfig());
  run(loop, 6.0);
  EXPECT_EQ(loop.status(), "hard_stop");
  EXPECT_NE(loop.reason().find("stale feedback"), std::string::npos) << loop.reason();
  EXPECT_FALSE(bus.enabled());
}

TEST(ControlLoop, MotorFaultIsHardStop)
{
  SimBusOptions o = simArm();
  o.fault_joint = 4;
  o.fault_after_s = 2.0;
  SimBus bus(o);
  ControlLoop loop(bus, tunedConfig());
  run(loop, 6.0);
  EXPECT_EQ(loop.status(), "hard_stop");
  EXPECT_NE(loop.reason().find("motor fault J5"), std::string::npos) << loop.reason();
  EXPECT_FALSE(bus.enabled());
}

TEST(ControlLoop, RejectsUnsafeConfiguration)
{
  SimBus bus(simArm());
  LoopConfig c = tunedConfig();
  c.filter_wn = 200.0;  // 0.2 per cycle at 1 kHz
  EXPECT_THROW(ControlLoop(bus, c), std::invalid_argument);
  c = tunedConfig();
  c.driver_kd[0] = 6.0;  // outside the MIT range
  EXPECT_THROW(ControlLoop(bus, c), std::invalid_argument);
}
