// Pinocchio with the CppADCodeGen scalar: M (CRBA), C (Coriolis matrix), G (gravity) of one arm.
// Other URDF joints (other arm, fingers) are fixed at 0.
#include "model_ad.hpp"

#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <memory>

namespace openarm_codegen
{
namespace
{
std::unique_ptr<pinocchio::ModelTpl<ADCG>> g_model;
pinocchio::Model g_model_double;
}

void loadModel(const std::string & urdf_path)
{
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  g_model = std::make_unique<pinocchio::ModelTpl<ADCG>>(model.cast<ADCG>());
  g_model_double = model;
}

std::vector<double> positionLimits(const std::string & arm, bool upper)
{
  std::vector<double> out;
  for (int j = 1; j <= kJoints; ++j) {
    const auto & joint = g_model_double.joints[g_model_double.getJointId(
          "openarm_" + arm + "_joint" + std::to_string(j))];
    out.push_back(upper ? g_model_double.upperPositionLimit[joint.idx_q()] :
      g_model_double.lowerPositionLimit[joint.idx_q()]);
  }
  return out;
}

std::vector<ADCG> modelTerms(const std::string & arm, const std::vector<ADCG> & x)
{
  const auto & model = *g_model;
  std::vector<int> iq, iv;
  for (int j = 1; j <= kJoints; ++j) {
    const auto & joint = model.joints[model.getJointId("openarm_" + arm + "_joint" + std::to_string(j))];
    iq.push_back(joint.idx_q());
    iv.push_back(joint.idx_v());
  }
  pinocchio::DataTpl<ADCG> data(model);
  Eigen::Matrix<ADCG, Eigen::Dynamic, 1> q = pinocchio::neutral(model);
  Eigen::Matrix<ADCG, Eigen::Dynamic, 1> v = Eigen::Matrix<ADCG, Eigen::Dynamic, 1>::Zero(model.nv);
  for (int j = 0; j < kJoints; ++j) {
    q[iq[j]] = x[j];
    v[iv[j]] = x[kJoints + j];
  }
  pinocchio::crba(model, data, q);  // upper triangle of M
  pinocchio::computeCoriolisMatrix(model, data, q, v);
  pinocchio::computeGeneralizedGravity(model, data, q);
  std::vector<ADCG> y(kOut);
  for (int r = 0; r < kJoints; ++r) {
    for (int c = 0; c < kJoints; ++c) {
      y[r * kJoints + c] = data.M(std::min(iv[r], iv[c]), std::max(iv[r], iv[c]));
      y[kJoints * kJoints + r * kJoints + c] = data.C(iv[r], iv[c]);
    }
    y[2 * kJoints * kJoints + r] = data.g(iv[r]);
  }
  return y;
}
}  // namespace openarm_codegen
