/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include "OpenArmPinocchioModel.h"

#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <memory>

namespace ct {
namespace models {
namespace OpenArm {
namespace codegen {

namespace {
pinocchio::Model modelDouble;
std::unique_ptr<pinocchio::ModelTpl<ADCGScalar>> modelAD;

std::vector<pinocchio::JointIndex> armJoints(const std::string& arm)
{
    std::vector<pinocchio::JointIndex> ids;
    for (size_t j = 1; j <= NJOINTS_ARM; ++j)
        ids.push_back(modelDouble.getJointId("openarm_" + arm + "_joint" + std::to_string(j)));
    return ids;
}
}  // namespace

void loadUrdf(const std::string& urdfPath)
{
    pinocchio::urdf::buildModel(urdfPath, modelDouble);
    modelAD.reset(new pinocchio::ModelTpl<ADCGScalar>(modelDouble.cast<ADCGScalar>()));
}

std::vector<ADCGScalar> modelTerms(const std::string& arm, const std::vector<ADCGScalar>& x)
{
    const auto& model = *modelAD;
    const auto ids = armJoints(arm);
    pinocchio::DataTpl<ADCGScalar> data(model);
    Eigen::Matrix<ADCGScalar, Eigen::Dynamic, 1> q = pinocchio::neutral(model);
    Eigen::Matrix<ADCGScalar, Eigen::Dynamic, 1> v = Eigen::Matrix<ADCGScalar, Eigen::Dynamic, 1>::Zero(model.nv);
    std::vector<int> iv;
    for (size_t j = 0; j < NJOINTS_ARM; ++j)
    {
        q[model.joints[ids[j]].idx_q()] = x[j];
        v[model.joints[ids[j]].idx_v()] = x[NJOINTS_ARM + j];
        iv.push_back(model.joints[ids[j]].idx_v());
    }
    pinocchio::crba(model, data, q);  // upper triangle of M
    pinocchio::computeCoriolisMatrix(model, data, q, v);
    pinocchio::computeGeneralizedGravity(model, data, q);

    std::vector<ADCGScalar> y(OUT_DIM);
    for (size_t r = 0; r < NJOINTS_ARM; ++r)
    {
        for (size_t c = 0; c < NJOINTS_ARM; ++c)
        {
            y[r * NJOINTS_ARM + c] = data.M(std::min(iv[r], iv[c]), std::max(iv[r], iv[c]));
            y[NJOINTS_ARM * NJOINTS_ARM + r * NJOINTS_ARM + c] = data.C(iv[r], iv[c]);
        }
        y[2 * NJOINTS_ARM * NJOINTS_ARM + r] = data.g(iv[r]);
    }
    return y;
}

std::vector<double> positionLimits(const std::string& arm, bool upper)
{
    std::vector<double> limits;
    for (const auto id : armJoints(arm))
    {
        const auto iq = modelDouble.joints[id].idx_q();
        limits.push_back(upper ? modelDouble.upperPositionLimit[iq] : modelDouble.lowerPositionLimit[iq]);
    }
    return limits;
}

}  // namespace codegen
}  // namespace OpenArm
}  // namespace models
}  // namespace ct
