// OpenArm joint-space model from generated code (no Pinocchio at run time).
//
// generated/OpenArm{Right,Left}Model.{h,cpp} are written offline by openarm_codegen: Pinocchio with
// the CppADCodeGen scalar + control-toolbox code generation, from the official URDF. Each evaluates,
// for one arm, M(q), C(q, dq) (Coriolis matrix) and G(q). Fingers are fixed at 0 (closed); the two
// arms hang from the fixed body, so a two-arm model is block diagonal.
#pragma once

#include <ct/core/core.h>

#include <memory>
#include <string>
#include <vector>

namespace openarm_control
{

template<size_t NJ>
using JointVector = Eigen::Matrix<double, NJ, 1>;
template<size_t NJ>
using JointMatrix = Eigen::Matrix<double, NJ, NJ>;
template<size_t NJ>
using State = ct::core::StateVector<2 * NJ>;      // x = [q; dq]
template<size_t NJ>
using Torque = ct::core::ControlVector<NJ>;

// M(q), C(q, dq), G(q) at one joint state. Equation of motion: M ddq + C dq + G = tau.
template<size_t NJ>
struct ModelTerms
{
  JointMatrix<NJ> M = JointMatrix<NJ>::Zero();
  JointMatrix<NJ> C = JointMatrix<NJ>::Zero();
  JointVector<NJ> G = JointVector<NJ>::Zero();
};

template<size_t NJ>
class OpenArmModel
{
public:
  // joint_names: NJ joints, each "openarm_<right|left>_joint<1..7>"; an arm that appears must appear
  // with all 7 joints. Order is free (vectors follow it). Throws std::invalid_argument otherwise.
  explicit OpenArmModel(const std::vector<std::string> & joint_names);
  OpenArmModel(const OpenArmModel & other);
  ~OpenArmModel();

  // Throws std::invalid_argument for non-finite input.
  ModelTerms<NJ> compute(const JointVector<NJ> & q, const JointVector<NJ> & dq) const;
  const std::vector<std::string> & jointNames() const;
  // URDF position limits of the joints (from the generated description).
  JointVector<NJ> lowerLimits() const;
  JointVector<NJ> upperLimits() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// The model as a control-toolbox system for ct_optcon solvers (LQR, iLQR, MPC):
// dx = [dq; M^-1 (u - C dq - G)].
template<size_t NJ>
class OpenArmDynamics : public ct::core::ControlledSystem<2 * NJ, NJ>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit OpenArmDynamics(const std::vector<std::string> & joint_names);
  OpenArmDynamics * clone() const override {return new OpenArmDynamics(*this);}
  void computeControlledDynamics(
    const State<NJ> & state, const double & t, const Torque<NJ> & control,
    ct::core::StateVector<2 * NJ> & derivative) override;
  const OpenArmModel<NJ> & model() const {return model_;}

private:
  OpenArmModel<NJ> model_;
};

extern template class OpenArmModel<7>;
extern template class OpenArmModel<14>;
extern template class OpenArmDynamics<7>;
extern template class OpenArmDynamics<14>;

}  // namespace openarm_control
