#include "openarm_controller/sim_bus.hpp"

#include <ct/models/OpenArm/OpenArm.h>

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace openarm_controller
{

namespace
{
// openarm_can 1.4.0 encoding ranges (DM8009, DM8009, DM4340, DM4340, DM4310 x3).
constexpr double kPMax = 12.5;
constexpr std::array<double, kJoints> kVMax{45, 45, 10, 10, 30, 30, 30};
constexpr std::array<double, kJoints> kTMax{54, 54, 28, 28, 10, 10, 10};

double quantize(double x, double lo, double hi, int bits)
{
  const double span = hi - lo;
  const double levels = static_cast<double>((1 << bits) - 1);
  const double u = std::round((std::clamp(x, lo, hi) - lo) / span * levels);
  return lo + u * span / levels;
}

std::vector<std::string> armJoints(const std::string & arm)
{
  if (arm != "left" && arm != "right") {throw std::invalid_argument("sim: arm must be left or right");}
  std::vector<std::string> names;
  for (int j = 1; j <= kJoints; ++j) {names.push_back("openarm_" + arm + "_joint" + std::to_string(j));}
  return names;
}
}  // namespace

struct SimBus::Impl
{
  explicit Impl(SimBusOptions o)
  : options(std::move(o)), model(armJoints(options.arm))
  {
    lower = model.jointLowerLimit();
    upper = model.jointUpperLimit();
    q = options.q0.cwiseMax(lower).cwiseMin(upper);
    dq.setZero();
    tau_motor.setZero();
  }

  void step(double dt)
  {
    if (!enabled && options.hold_when_disabled) {
      dq.setZero();
      tau_motor.setZero();
      time += dt;
      return;
    }
    Vec7 tau = Vec7::Zero();
    if (enabled) {
      for (int j = 0; j < kJoints; ++j) {
        const double t = last.kp[j] * (last.q[j] - q[j]) + last.kd[j] * (last.dq[j] - dq[j]) + last.tau[j];
        tau[j] = std::clamp(t, -kTMax[j], kTMax[j]);
      }
    }
    tau_motor = tau;
    model.computeTerms(q, dq, terms);
    const Vec7 friction = options.coulomb_nm.cwiseProduct((dq / 0.01).array().tanh().matrix()) +
      options.viscous_nms.cwiseProduct(dq);
    const Vec7 ddq = terms.M.llt().solve(tau - friction - terms.C * dq - terms.G);
    dq += dt * ddq;
    q += dt * dq;
    for (int j = 0; j < kJoints; ++j) {  // mechanical stops
      if (q[j] < lower[j]) {q[j] = lower[j]; dq[j] = std::max(0.0, dq[j]);}
      if (q[j] > upper[j]) {q[j] = upper[j]; dq[j] = std::min(0.0, dq[j]);}
    }
    time += dt;
  }

  void advance()
  {
    const int n = std::max(1, static_cast<int>(std::lround(options.cycle_s / options.step_s)));
    for (int k = 0; k < n; ++k) {step(options.cycle_s / n);}
    if (options.stale_after_s < 0.0 || time < options.stale_after_s) {last_reply = time;}
  }

  SimBusOptions options;
  ct::models::OpenArm::OpenArmModel<kJoints> model;
  ct::models::OpenArm::OpenArmModel<kJoints>::model_terms_t terms;
  Vec7 lower, upper;
  Vec7 q, dq, tau_motor;
  MotorCommand last;
  bool enabled = false;
  int enable_calls = 0;
  double time = 0.0;
  double last_reply = 0.0;
};

SimBus::SimBus(SimBusOptions options)
: impl_(std::make_unique<Impl>(std::move(options)))
{
}

SimBus::~SimBus()
{
}

void SimBus::enable()
{
  impl_->enabled = true;
  impl_->last = MotorCommand{};  // the Damiao driver starts from a zero command after enable
  ++impl_->enable_calls;
}

void SimBus::disable()
{
  impl_->enabled = false;
}

void SimBus::refresh()
{
  impl_->advance();
}

void SimBus::command(const MotorCommand & c)
{
  MotorCommand q;  // what arrives after the MIT encoding
  for (int j = 0; j < kJoints; ++j) {
    q.q[j] = quantize(c.q[j], -kPMax, kPMax, 16);
    q.dq[j] = quantize(c.dq[j], -kVMax[j], kVMax[j], 12);
    q.kp[j] = quantize(c.kp[j], 0.0, 500.0, 12);
    q.kd[j] = quantize(c.kd[j], 0.0, 5.0, 12);
    q.tau[j] = quantize(c.tau[j], -kTMax[j], kTMax[j], 12);
  }
  if (impl_->enabled) {impl_->last = q;}
  impl_->advance();
}

ArmState SimBus::read()
{
  ArmState s;
  const bool faulted = impl_->options.fault_joint >= 0 && impl_->options.fault_after_s >= 0.0 &&
    impl_->time >= impl_->options.fault_after_s;
  for (int j = 0; j < kJoints; ++j) {
    s.q[j] = quantize(impl_->q[j], -kPMax, kPMax, 16);
    s.dq[j] = quantize(impl_->dq[j], -kVMax[j], kVMax[j], 12);
    s.tau[j] = quantize(impl_->tau_motor[j], -kTMax[j], kTMax[j], 12);
    s.age_s[j] = impl_->time - impl_->last_reply;
    s.enabled[j] = impl_->enabled;
    s.error[j] = (faulted && j == impl_->options.fault_joint) ? 0x9 : (impl_->enabled ? 1 : 0);
  }
  return s;
}

double SimBus::now() const
{
  return impl_->time;
}

bool SimBus::realtime() const
{
  return false;
}

int SimBus::enableCalls() const
{
  return impl_->enable_calls;
}

bool SimBus::enabled() const
{
  return impl_->enabled;
}

Vec7 SimBus::trueQ() const
{
  return impl_->q;
}

}  // namespace openarm_controller
