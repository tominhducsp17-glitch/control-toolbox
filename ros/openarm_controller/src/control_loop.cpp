#include "openarm_controller/control_loop.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace openarm_controller
{

namespace
{
double smoothstep(double x)
{
  x = std::clamp(x, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

std::string jointList(const std::vector<int> & joints)
{
  std::ostringstream out;
  for (std::size_t k = 0; k < joints.size(); ++k) {out << (k ? "," : "") << "J" << joints[k] + 1;}
  return out.str();
}

std::vector<std::string> armJoints(const std::string & arm)
{
  if (arm != "left" && arm != "right") {throw std::invalid_argument("arm must be left or right");}
  std::vector<std::string> names;
  for (int j = 1; j <= kJoints; ++j) {names.push_back("openarm_" + arm + "_joint" + std::to_string(j));}
  return names;
}

double quinticDuration(const Vec7 & a, const Vec7 & b, double max_speed, double min_duration)
{
  return std::max(min_duration, 1.875 * (b - a).cwiseAbs().maxCoeff() / max_speed);
}
}  // namespace

ControlLoop::ControlLoop(ArmBus & bus, const LoopConfig & config)
: bus_(bus), config_(config), period_(1.0 / config.rate_hz), filter_(config.filter_wn)
{
  if (!(config_.rate_hz > 0.0) || !(config_.blend_s > 0.0) || !(config_.ramp_s > 0.0) ||
    !(config_.ramp_down_s > 0.0) || !(config_.return_speed > 0.0))
  {
    throw std::invalid_argument("rate_hz, blend_s, ramp_s, ramp_down_s and return_speed must be > 0");
  }
  if (config_.filter_wn * period_ > 0.15) {
    throw std::invalid_argument("filter_wn / rate_hz must be <= 0.15 (the semi-implicit Euler filter rings above)");
  }
  if ((config_.hold_kp.array() < 0.0).any() || (config_.hold_kp.array() > 500.0).any() ||
    (config_.hold_kd.array() < 0.0).any() || (config_.hold_kd.array() > 5.0).any() ||
    (config_.driver_kd.array() < 0.0).any() || (config_.driver_kd.array() > 5.0).any())
  {
    throw std::invalid_argument("driver gains outside the MIT range (kp 0..500, kd 0..5)");
  }
  const Model model(armJoints(config_.arm));
  q_lower_ = model.jointLowerLimit();
  q_upper_ = model.jointUpperLimit();
  feedforward_ = std::make_unique<ct::nlc::ComputedTorqueController<kJoints, Model>>(model);
  full_ctc_ = std::make_unique<ct::nlc::FullComputedTorqueController<kJoints, Model>>(
    model, config_.ctc_kp, config_.ctc_kd);
  full_ctc_->setModelCorrection(config_.tau_bias);
  full_ctc_->setFrictionFeedforward(config_.friction_fc, config_.friction_fv, config_.friction_v);
  full_ctc_->setIntegralGain(config_.ki, config_.integral_limit);
  feedforward_->setTorqueLimit(config_.tau_limit);
  full_ctc_->setTorqueLimit(config_.tau_limit);
  // reserved before the loop: a growing std::vector copies all rows, which stalled the loop for 6-10 ms on the robot
  log_.reserve(static_cast<std::size_t>(config_.log_max_s * config_.rate_hz));
}

ControlLoop::~ControlLoop()
{
  // last line of defence: an exception between start() and the end of the loop still disables the motors
  if (phase_ != Phase::kConnect && phase_ != Phase::kDone) {
    try {
      bus_.disable();
    } catch (...) {
    }
  }
}

double ControlLoop::elapsed() const
{
  return bus_.now() - t0_;
}

bool ControlLoop::start()
{
  t0_ = bus_.now();
  while (elapsed() < config_.connect_s) {
    bus_.refresh();
    state_ = bus_.read();
    if (bus_.realtime()) {std::this_thread::sleep_for(std::chrono::duration<double>(period_));}
  }
  std::vector<int> bad;
  for (int j = 0; j < kJoints; ++j) {
    const bool ok = std::isfinite(state_.q[j]) && state_.age_s[j] < 0.05 &&
      state_.error[j] < kMotorFaultThreshold && !state_.enabled[j] &&
      state_.q[j] >= q_lower_[j] - config_.position_margin && state_.q[j] <= q_upper_[j] + config_.position_margin;
    if (!ok) {bad.push_back(j);}
  }
  if (!bad.empty()) {
    status_ = "not_started";
    reason_ = "connect check failed (no reply, fault, already enabled or outside the limits): " + jointList(bad);
    phase_ = Phase::kDone;
    return false;
  }
  q0_ = state_.q;
  prev_q_d_ = q0_;
  filter_.reset(q0_);
  last_ref_.q = q0_;
  status_ = "running";
  bus_.enable();
  enterPhase(Phase::kRamp, elapsed());
  return true;
}

void ControlLoop::setTarget(const Vec7 & q)
{
  if (!q.allFinite() || phase_ == Phase::kStopped || phase_ == Phase::kReturn || phase_ == Phase::kRampDown ||
    phase_ == Phase::kDone)
  {
    return;
  }
  // inside the URDF limits: the filter never takes the reference out of them
  target_ = q.cwiseMax(q_lower_).cwiseMin(q_upper_);
  has_target_ = true;
  target_t_ = elapsed();
}

void ControlLoop::requestStop()
{
  ++stop_requests_;
}

void ControlLoop::enterPhase(Phase phase, double t)
{
  phase_ = phase;
  phase_start_ = t;
}

void ControlLoop::softStop(const std::string & why, double t)
{
  if (reason_.empty()) {reason_ = why;}
  blend_from_ = blend(t);
  stop_t_ = t;
  // where the reference comes to rest: the filter, fed this fixed point, decelerates without a step in the command
  stop_target_ = (last_ref_.q + last_ref_.dq * (4.0 / config_.filter_wn)).cwiseMax(q_lower_).cwiseMin(q_upper_);
  full_ctc_->setIntegratorActive(false);
  enterPhase(Phase::kStopped, t);
}

void ControlLoop::hardStop(const std::string & why, double t)
{
  (void)t;
  reason_ = reason_.empty() ? why : reason_ + "; then " + why;
  status_ = "hard_stop";
  bus_.disable();
  phase_ = Phase::kDone;
}

double ControlLoop::blend(double t) const
{
  const double tp = t - phase_start_;
  switch (phase_) {
    case Phase::kTrack: return smoothstep(tp / config_.blend_s);
    case Phase::kStopped:
    case Phase::kReturn: return blend_from_ * (1.0 - smoothstep((t - stop_t_) / config_.blend_s));
    default: return 0.0;
  }
}

bool ControlLoop::checkSafety(double t)
{
  std::vector<int> stale, fault, err, fast, out;
  for (int j = 0; j < kJoints; ++j) {
    if (!(state_.age_s[j] <= config_.max_age)) {stale.push_back(j);}
    if (state_.error[j] >= kMotorFaultThreshold) {fault.push_back(j);}
    if (std::abs(state_.q[j] - prev_q_d_[j]) > config_.max_error) {err.push_back(j);}
    if (std::abs(state_.dq[j]) > config_.max_speed) {fast.push_back(j);}
    if (state_.q[j] < q_lower_[j] - config_.position_margin || state_.q[j] > q_upper_[j] + config_.position_margin) {
      out.push_back(j);
    }
  }
  std::string hard, soft;
  if (!stale.empty()) {hard = "stale feedback " + jointList(stale);}
  if (!fault.empty()) {hard = "motor fault " + jointList(fault);}
  if (stop_requests_ >= 2) {hard = "second user stop";}
  if (!err.empty()) {soft = "tracking error > " + std::to_string(config_.max_error) + " rad " + jointList(err);}
  if (!fast.empty()) {soft = "speed > " + std::to_string(config_.max_speed) + " rad/s " + jointList(fast);}
  if (!out.empty()) {soft = "outside the position limits " + jointList(out);}
  const bool stopping = phase_ == Phase::kStopped || phase_ == Phase::kReturn || phase_ == Phase::kRampDown;
  const bool in_grace = stop_t_ >= 0.0 && t - stop_t_ < config_.abort_grace_s;
  if (hard.empty() && !soft.empty() && stopping && !in_grace) {hard = soft + " during the stop";}
  if (!hard.empty()) {
    hardStop(hard, t);
    return false;
  }
  if (!soft.empty() && !stopping) {softStop(soft, t);}
  return true;
}

ControlLoop::Reference ControlLoop::reference(double t, double tp, double & scale)
{
  Reference ref;
  scale = 1.0;
  switch (phase_) {
    case Phase::kRamp:
      scale = smoothstep(tp / config_.ramp_s);
      ref.q = q0_;
      break;
    case Phase::kHold:
      ref.q = q0_;
      break;
    case Phase::kTrack:
    case Phase::kStopped:
      filter_.update(phase_ == Phase::kTrack ? target_ : stop_target_, period_);
      ref.q = filter_.getPosition();
      ref.dq = filter_.getVelocity();
      ref.ddq = filter_.getAcceleration();
      break;
    case Phase::kReturn: {
        // quintic from the reference at the stop (position, velocity, acceleration) to rest at the start pose
        const double T = return_s_;
        const double x = std::clamp(tp, 0.0, T);
        const Vec7 p0 = return_from_.q, v0 = return_from_.dq, a0 = return_from_.ddq, d = q0_ - p0;
        const Vec7 c3 = (20.0 * d - 12.0 * T * v0 - 3.0 * T * T * a0) / (2.0 * T * T * T);
        const Vec7 c4 = (-30.0 * d + 16.0 * T * v0 + 3.0 * T * T * a0) / (2.0 * T * T * T * T);
        const Vec7 c5 = (12.0 * d - 6.0 * T * v0 - T * T * a0) / (2.0 * T * T * T * T * T);
        ref.q = p0 + x * v0 + 0.5 * x * x * a0 + x * x * x * c3 + x * x * x * x * c4 + x * x * x * x * x * c5;
        ref.dq = v0 + x * a0 + 3.0 * x * x * c3 + 4.0 * x * x * x * c4 + 5.0 * x * x * x * x * c5;
        ref.ddq = a0 + 6.0 * x * c3 + 12.0 * x * x * c4 + 20.0 * x * x * x * c5;
        break;
      }
    case Phase::kRampDown:
      scale = 1.0 - smoothstep(tp / config_.ramp_down_s);
      ref.q = q0_;
      break;
    default:
      ref.q = q0_;
      break;
  }
  (void)t;
  return ref;
}

bool ControlLoop::step()
{
  if (phase_ == Phase::kConnect || phase_ == Phase::kDone) {return false;}
  const auto cycle_begin = std::chrono::steady_clock::now();
  const double t = elapsed();
  try {
    state_ = bus_.read();
    if (!checkSafety(t)) {return false;}

    // ---- user stop: return to the start pose, then disable ----
    if (stop_requests_ == 1 && phase_ != Phase::kReturn && phase_ != Phase::kRampDown) {
      if (reason_.empty()) {reason_ = "user stop";}
      if (phase_ != Phase::kStopped) {
        blend_from_ = blend(t);
        stop_t_ = t;
      }
      return_from_ = last_ref_;
      return_s_ = quinticDuration(last_ref_.q, q0_, config_.return_speed, config_.min_move_s) +
        2.0 * last_ref_.dq.cwiseAbs().maxCoeff() / config_.return_speed;
      full_ctc_->setIntegratorActive(false);
      enterPhase(Phase::kReturn, t);
    }

    // ---- phase transitions (each with the time in the phase it leaves) ----
    if (phase_ == Phase::kRamp && t - phase_start_ >= config_.ramp_s) {enterPhase(Phase::kHold, t);}
    if (phase_ == Phase::kHold && has_target_) {
      full_ctc_->resetIntegral();
      full_ctc_->setIntegratorActive(true);
      enterPhase(Phase::kTrack, t);
    }
    if (phase_ == Phase::kReturn && t - phase_start_ >= return_s_) {enterPhase(Phase::kRampDown, t);}
    if (phase_ == Phase::kRampDown && t - phase_start_ >= config_.ramp_down_s) {
      bus_.disable();
      status_ = reason_ == "user stop" ? "completed" : "stopped";
      phase_ = Phase::kDone;
      return false;
    }
    const double tp = t - phase_start_;

    // ---- reference and torque ----
    double scale = 1.0;
    const Reference ref = reference(t, tp, scale);
    const double w = blend(t);
    ct::core::StateVector<2 * kJoints> x;
    x << state_.q, state_.dq;
    Vec7 tau = Vec7::Zero();
    if (w < 1.0) {  // driver PD: model feedforward of the reference (+ correction, friction)
      feedforward_->setReference(ref.q, ref.dq, ref.ddq);
      ct::core::ControlVector<kJoints> u;
      feedforward_->computeControl(x, t, u);
      tau += (1.0 - w) * (Vec7(u) + full_ctc_->feedforwardCorrection(ref.dq));
    }
    full_ctc_->setReference(ref.q, ref.dq, ref.ddq);
    ct::core::ControlVector<kJoints> u_ctc;
    full_ctc_->computeControl(x, t, u_ctc);  // every cycle in the track phase: the integral sees each step
    tau += w * Vec7(u_ctc);
    const Vec7 limited = tau.cwiseMax(-config_.tau_limit).cwiseMin(config_.tau_limit);
    const bool clamped = (limited - tau).cwiseAbs().maxCoeff() > 1e-9;
    clamped_cycles_ += clamped ? 1 : 0;

    MotorCommand command;
    command.q = ref.q;
    command.dq = ref.dq;
    command.kp = scale * (1.0 - w) * config_.hold_kp;
    command.kd = scale * ((1.0 - w) * config_.hold_kd + w * config_.driver_kd);
    command.tau = scale * limited;
    bus_.command(command);
    prev_q_d_ = ref.q;
    last_ref_ = ref;

    row_.t = t;
    row_.phase = static_cast<int>(phase_);
    row_.scale = scale;
    row_.track_t = phase_ == Phase::kTrack ? tp : -1.0;
    row_.q_d = ref.q;
    row_.dq_d = ref.dq;
    row_.ddq_d = ref.ddq;
    row_.q = state_.q;
    row_.dq = state_.dq;
    row_.tau_measured = state_.tau;
    row_.tau_sent = command.tau;
    row_.target = has_target_ ? target_ : q0_;
    row_.max_age = *std::max_element(state_.age_s.begin(), state_.age_s.end());
    row_.loop_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - cycle_begin).count();
    row_.clamped = clamped;
    row_.blend = w;
    if (log_.size() < log_.capacity()) {log_.push_back(row_);}
  } catch (const std::exception & e) {
    hardStop(std::string("exception: ") + e.what(), t);
    return false;
  }
  return true;
}

void ControlLoop::waitNextCycle()
{
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(period_));
  if (!ticking_) {
    next_tick_ = std::chrono::steady_clock::now();
    ticking_ = true;
  }
  next_tick_ += period;
  if (std::chrono::steady_clock::now() > next_tick_) {
    ++overruns_;
    next_tick_ = std::chrono::steady_clock::now();
  } else {
    std::this_thread::sleep_until(next_tick_);
  }
}

Phase ControlLoop::phase() const {return phase_;}
const std::string & ControlLoop::status() const {return status_;}
const std::string & ControlLoop::reason() const {return reason_;}
const ArmState & ControlLoop::state() const {return state_;}
const LogRow & ControlLoop::lastRow() const {return row_;}
const std::vector<LogRow> & ControlLoop::log() const {return log_;}
int ControlLoop::overruns() const {return overruns_;}
int ControlLoop::clampedCycles() const {return clamped_cycles_;}

double ControlLoop::secondsSinceTarget() const
{
  return target_t_ < 0.0 ? std::numeric_limits<double>::infinity() : elapsed() - target_t_;
}

void ControlLoop::writeLog(const std::string & path) const
{
  std::ofstream out(path);
  if (!out) {throw std::runtime_error("cannot write " + path);}
  out << "t,phase,scale,track_t";
  for (const char * name : {"q_d", "dq_d", "ddq_d", "q", "dq", "tau_measured", "tau_ff"}) {
    for (int j = 1; j <= kJoints; ++j) {out << ',' << name << j;}
  }
  out << ",max_age,loop_us,clamped,blend";
  for (int j = 1; j <= kJoints; ++j) {out << ",q_src" << j;}
  out << '\n' << std::setprecision(9);
  for (const auto & r : log_) {
    out << r.t << ',' << r.phase << ',' << r.scale << ',' << r.track_t;
    for (const Vec7 * v : {&r.q_d, &r.dq_d, &r.ddq_d, &r.q, &r.dq, &r.tau_measured, &r.tau_sent}) {
      for (int j = 0; j < kJoints; ++j) {out << ',' << (*v)[j];}
    }
    out << ',' << r.max_age << ',' << r.loop_us << ',' << (r.clamped ? 1 : 0) << ',' << r.blend;
    for (int j = 0; j < kJoints; ++j) {out << ',' << r.target[j];}
    out << '\n';
  }
}

}  // namespace openarm_controller
