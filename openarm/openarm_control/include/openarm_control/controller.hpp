// Controllers: one virtual base, one common input.
//
// Every controller receives the same ControllerInput: measured state (q, dq), reference
// (q_d, dq_d, ddq_d) and the model terms M, C, G evaluated at both the measured and the reference
// state (computed by the caller with OpenArmModel, i.e. generated code). A new controller derives from
// ControllerBase and overrides computeTorque(); nodes and the model stay unchanged.
#pragma once

#include "openarm_control/openarm_model.hpp"

#include <memory>
#include <string>
#include <vector>

namespace openarm_control
{

template<size_t NJ>
struct ControllerInput
{
  double t = 0.0;
  // measured
  JointVector<NJ> q = JointVector<NJ>::Zero();
  JointVector<NJ> dq = JointVector<NJ>::Zero();
  // reference
  JointVector<NJ> q_d = JointVector<NJ>::Zero();
  JointVector<NJ> dq_d = JointVector<NJ>::Zero();
  JointVector<NJ> ddq_d = JointVector<NJ>::Zero();
  // model: M, C, G at the measured state (q, dq) and at the reference state (q_d, dq_d)
  ModelTerms<NJ> measured;
  ModelTerms<NJ> reference;
};

// Fills the model terms of `in` from its states (both evaluations use the generated model).
template<size_t NJ>
void computeModelTerms(const OpenArmModel<NJ> & model, ControllerInput<NJ> & in);

template<size_t NJ>
class ControllerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual ~ControllerBase() = default;

  // Override in each controller: joint torque [N m] from the common input.
  virtual JointVector<NJ> computeTorque(const ControllerInput<NJ> & in) = 0;
  virtual std::string name() const = 0;
  virtual std::unique_ptr<ControllerBase> clone() const = 0;

  // Called by the node: checks the input, calls computeTorque(), clamps to torque_limit.
  JointVector<NJ> compute(const ControllerInput<NJ> & in);
  // |tau_i| <= limit_i; empty = no clamp. Throws std::invalid_argument for a wrong size or limit <= 0.
  void setTorqueLimit(const std::vector<double> & limit);
  bool lastClamped() const {return clamped_;}

private:
  JointVector<NJ> limit_ = JointVector<NJ>::Constant(std::numeric_limits<double>::infinity());
  bool clamped_ = false;
};

// Model part of computed-torque control, for a driver that closes the PD loop:
// tau_ff = M(q_d) ddq_d + C(q_d, dq_d) dq_d + G(q_d)   (reference terms only).
template<size_t NJ>
class CtcFeedforward : public ControllerBase<NJ>
{
public:
  JointVector<NJ> computeTorque(const ControllerInput<NJ> & in) override
  {
    return in.reference.M * in.ddq_d + in.reference.C * in.dq_d + in.reference.G;
  }
  std::string name() const override {return "ctc_feedforward";}
  std::unique_ptr<ControllerBase<NJ>> clone() const override {return std::make_unique<CtcFeedforward>(*this);}
};

// Gravity compensation at the reference: tau = G(q_d).
template<size_t NJ>
class GravityCompensation : public ControllerBase<NJ>
{
public:
  JointVector<NJ> computeTorque(const ControllerInput<NJ> & in) override {return in.reference.G;}
  std::string name() const override {return "gravity_compensation";}
  std::unique_ptr<ControllerBase<NJ>> clone() const override {return std::make_unique<GravityCompensation>(*this);}
};

// "ctc_feedforward" | "gravity_compensation". Register new controllers here.
template<size_t NJ>
std::unique_ptr<ControllerBase<NJ>> makeController(const std::string & type);
std::vector<std::string> availableControllers();

// ddq_d from a stream of dq_d samples: finite difference, first-order low-pass, magnitude clamp.
template<size_t NJ>
class AccelerationEstimator
{
public:
  AccelerationEstimator(double cutoff_hz, double max_abs_acceleration);
  JointVector<NJ> update(double t, const JointVector<NJ> & dq);
  void reset();

private:
  double cutoff_hz_, max_abs_;
  bool has_previous_ = false;
  double t_prev_ = 0.0;
  JointVector<NJ> dq_prev_ = JointVector<NJ>::Zero();
  JointVector<NJ> estimate_ = JointVector<NJ>::Zero();
};

// A ControllerBase used as a control-toolbox controller (ct::core::Controller), e.g. to simulate it
// with ct::core integrators: computes the model terms at the state and calls the controller.
template<size_t NJ>
class CtControllerAdapter : public ct::core::Controller<2 * NJ, NJ>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CtControllerAdapter(std::shared_ptr<ControllerBase<NJ>> controller, const OpenArmModel<NJ> & model);
  CtControllerAdapter(const CtControllerAdapter & other);
  CtControllerAdapter * clone() const override {return new CtControllerAdapter(*this);}
  void setReference(const JointVector<NJ> & q_d, const JointVector<NJ> & dq_d, const JointVector<NJ> & ddq_d);
  void computeControl(const State<NJ> & state, const double & t, Torque<NJ> & control) override;

private:
  std::shared_ptr<ControllerBase<NJ>> controller_;
  OpenArmModel<NJ> model_;
  ControllerInput<NJ> input_;
};

extern template class ControllerBase<7>;
extern template class ControllerBase<14>;
extern template class AccelerationEstimator<7>;
extern template class AccelerationEstimator<14>;
extern template class CtControllerAdapter<7>;
extern template class CtControllerAdapter<14>;

}  // namespace openarm_control
