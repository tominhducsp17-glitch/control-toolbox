// Transitional adapter: the OpenArm model now lives in ct_models (ct::models::OpenArm, generated code).
// This header keeps the openarm_control API until the controllers move to ct_nlc.
#pragma once

#include <ct/models/OpenArm/OpenArm.h>

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
using State = ct::core::StateVector<2 * NJ>;
template<size_t NJ>
using Torque = ct::core::ControlVector<NJ>;
template<size_t NJ>
using ModelTerms = ct::models::OpenArm::ModelTerms<NJ>;

template<size_t NJ>
class OpenArmModel
{
public:
  explicit OpenArmModel(const std::vector<std::string> & joint_names)
  : model_(std::make_shared<ct::models::OpenArm::OpenArmModel<NJ>>(joint_names)) {}
  OpenArmModel(const OpenArmModel & other)
  : model_(other.model_->clone()) {}
  ModelTerms<NJ> compute(const JointVector<NJ> & q, const JointVector<NJ> & dq) const
  {
    return model_->computeTerms(q, dq);
  }
  const std::vector<std::string> & jointNames() const {return model_->jointNames();}
  JointVector<NJ> lowerLimits() const {return model_->jointLowerLimit();}
  JointVector<NJ> upperLimits() const {return model_->jointUpperLimit();}

private:
  std::shared_ptr<ct::models::OpenArm::OpenArmModel<NJ>> model_;
};

template<size_t NJ>
using OpenArmDynamics = ct::models::OpenArm::OpenArmSystem<NJ>;

}  // namespace openarm_control
