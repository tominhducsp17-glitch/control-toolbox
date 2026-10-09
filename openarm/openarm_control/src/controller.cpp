#include "openarm_control/controller.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace openarm_control
{

template<size_t NJ>
void computeModelTerms(const OpenArmModel<NJ> & model, ControllerInput<NJ> & in)
{
  in.measured = model.compute(in.q, in.dq);
  in.reference = model.compute(in.q_d, in.dq_d);
}

template<size_t NJ>
JointVector<NJ> ControllerBase<NJ>::compute(const ControllerInput<NJ> & in)
{
  if (!in.q.allFinite() || !in.dq.allFinite() || !in.q_d.allFinite() || !in.dq_d.allFinite() ||
    !in.ddq_d.allFinite())
  {
    throw std::invalid_argument("controller input must be finite");
  }
  const JointVector<NJ> raw = computeTorque(in);
  if (!raw.allFinite()) {throw std::runtime_error(name() + ": non-finite torque");}
  const JointVector<NJ> out = raw.cwiseMax(-limit_).cwiseMin(limit_);
  clamped_ = (out - raw).cwiseAbs().maxCoeff() > 0.0;
  return out;
}

template<size_t NJ>
void ControllerBase<NJ>::setTorqueLimit(const std::vector<double> & limit)
{
  if (limit.empty()) {
    limit_.setConstant(std::numeric_limits<double>::infinity());
    return;
  }
  if (limit.size() != NJ) {throw std::invalid_argument("torque_limit must have one value per joint");}
  for (size_t j = 0; j < NJ; ++j) {
    if (!(limit[j] > 0.0)) {throw std::invalid_argument("torque_limit must be > 0");}
    limit_[j] = limit[j];
  }
}

template<size_t NJ>
std::unique_ptr<ControllerBase<NJ>> makeController(const std::string & type)
{
  if (type == "ctc_feedforward") {return std::make_unique<CtcFeedforward<NJ>>();}
  if (type == "gravity_compensation") {return std::make_unique<GravityCompensation<NJ>>();}
  throw std::invalid_argument("unknown controller type '" + type + "'");
}

std::vector<std::string> availableControllers() {return {"ctc_feedforward", "gravity_compensation"};}

template<size_t NJ>
AccelerationEstimator<NJ>::AccelerationEstimator(double cutoff_hz, double max_abs_acceleration)
: cutoff_hz_(cutoff_hz), max_abs_(max_abs_acceleration)
{
  if (!(cutoff_hz > 0.0) || !(max_abs_acceleration > 0.0)) {
    throw std::invalid_argument("AccelerationEstimator: cutoff and limit must be > 0");
  }
}

template<size_t NJ>
JointVector<NJ> AccelerationEstimator<NJ>::update(double t, const JointVector<NJ> & dq)
{
  if (has_previous_ && t > t_prev_) {
    const double dt = t - t_prev_;
    const JointVector<NJ> raw = (dq - dq_prev_) / dt;
    const double alpha = 1.0 - std::exp(-2.0 * M_PI * cutoff_hz_ * dt);
    estimate_ += alpha * (raw - estimate_);
    estimate_ = estimate_.cwiseMax(-max_abs_).cwiseMin(max_abs_);
  }
  if (!has_previous_ || t > t_prev_) {
    has_previous_ = true;
    t_prev_ = t;
    dq_prev_ = dq;
  }
  return estimate_;
}

template<size_t NJ>
void AccelerationEstimator<NJ>::reset()
{
  has_previous_ = false;
  estimate_.setZero();
}

template<size_t NJ>
CtControllerAdapter<NJ>::CtControllerAdapter(
  std::shared_ptr<ControllerBase<NJ>> controller, const OpenArmModel<NJ> & model)
: controller_(std::move(controller)), model_(model) {}

template<size_t NJ>
CtControllerAdapter<NJ>::CtControllerAdapter(const CtControllerAdapter & other)
: ct::core::Controller<2 * NJ, NJ>(other), controller_(other.controller_->clone()), model_(other.model_),
  input_(other.input_) {}

template<size_t NJ>
void CtControllerAdapter<NJ>::setReference(
  const JointVector<NJ> & q_d, const JointVector<NJ> & dq_d, const JointVector<NJ> & ddq_d)
{
  input_.q_d = q_d;
  input_.dq_d = dq_d;
  input_.ddq_d = ddq_d;
}

template<size_t NJ>
void CtControllerAdapter<NJ>::computeControl(const State<NJ> & state, const double & t, Torque<NJ> & control)
{
  input_.t = t;
  input_.q = state.template head<NJ>();
  input_.dq = state.template tail<NJ>();
  computeModelTerms(model_, input_);
  control = controller_->compute(input_);
}

template void computeModelTerms<7>(const OpenArmModel<7> &, ControllerInput<7> &);
template void computeModelTerms<14>(const OpenArmModel<14> &, ControllerInput<14> &);
template std::unique_ptr<ControllerBase<7>> makeController<7>(const std::string &);
template std::unique_ptr<ControllerBase<14>> makeController<14>(const std::string &);
template class ControllerBase<7>;
template class ControllerBase<14>;
template class AccelerationEstimator<7>;
template class AccelerationEstimator<14>;
template class CtControllerAdapter<7>;
template class CtControllerAdapter<14>;

}  // namespace openarm_control
