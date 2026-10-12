/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <set>
#include <stdexcept>

namespace ct {
namespace models {
namespace OpenArm {

namespace detail {
template <size_t NJOINTS>
std::vector<std::string> defaultJointNames()
{
    return NJOINTS == 14 ? urdfJointNames() : armJointNames("right");
}
}  // namespace detail

template <size_t NJOINTS>
OpenArmModel<NJOINTS>::OpenArmModel() : OpenArmModel(detail::defaultJointNames<NJOINTS>())
{
}

template <size_t NJOINTS>
OpenArmModel<NJOINTS>::OpenArmModel(const std::vector<std::string>& jointNames)
    : jointNames_(jointNames), hasLeft_(false), hasRight_(false), x_(Eigen::VectorXd::Zero(14))
{
    static_assert(NJOINTS == 7 || NJOINTS == 14, "OpenArmModel: 7 (one arm) or 14 (both arms) joints");
    if (jointNames_.size() != NJOINTS)
        throw std::invalid_argument("OpenArmModel: expected " + std::to_string(NJOINTS) + " joint names");
    if (std::set<std::string>(jointNames_.begin(), jointNames_.end()).size() != NJOINTS)
        throw std::invalid_argument("OpenArmModel: duplicate joint names");

    leftIndex_.fill(-1);
    rightIndex_.fill(-1);
    int nLeft = 0, nRight = 0;
    for (size_t k = 0; k < NJOINTS; ++k)
    {
        const std::string& name = jointNames_[k];
        const std::string left = "openarm_left_joint", right = "openarm_right_joint";
        const char last = name.empty() ? '\0' : name.back();
        const bool validNumber = last >= '1' && last <= '7';
        if (validNumber && name.size() == left.size() + 1 && name.compare(0, left.size(), left) == 0)
        {
            leftIndex_[last - '1'] = static_cast<int>(k);
            ++nLeft;
        }
        else if (validNumber && name.size() == right.size() + 1 && name.compare(0, right.size(), right) == 0)
        {
            rightIndex_[last - '1'] = static_cast<int>(k);
            ++nRight;
        }
        else
            throw std::invalid_argument("OpenArmModel: unknown joint " + name);
    }
    if ((nLeft != 0 && nLeft != 7) || (nRight != 0 && nRight != 7))
        throw std::invalid_argument("OpenArmModel: each arm must list all its joints J1..J7");
    hasLeft_ = nLeft == 7;
    hasRight_ = nRight == 7;
}

template <size_t NJOINTS>
OpenArmModel<NJOINTS>::OpenArmModel(const OpenArmModel& arg) : OpenArmModel(arg.jointNames_)
{
}

template <size_t NJOINTS>
OpenArmModel<NJOINTS>::~OpenArmModel()
{
}

template <size_t NJOINTS>
OpenArmModel<NJOINTS>* OpenArmModel<NJOINTS>::clone() const
{
    return new OpenArmModel<NJOINTS>(*this);
}

template <size_t NJOINTS>
template <class ArmModel>
void OpenArmModel<NJOINTS>::addArm(ArmModel& armModel,
    const std::array<int, 7>& index,
    const joint_vector_t& q,
    const joint_vector_t& dq,
    model_terms_t& terms)
{
    for (int j = 0; j < 7; ++j)
    {
        x_(j) = q(index[j]);
        x_(7 + j) = dq(index[j]);
    }
    const auto y = armModel.forwardZero(x_);  // [M row-major (49); C row-major (49); G (7)]
    for (int r = 0; r < 7; ++r)
    {
        for (int c = 0; c < 7; ++c)
        {
            terms.M(index[r], index[c]) = y(r * 7 + c);
            terms.C(index[r], index[c]) = y(49 + r * 7 + c);
        }
        terms.G(index[r]) = y(98 + r);
    }
}

template <size_t NJOINTS>
void OpenArmModel<NJOINTS>::computeTerms(const joint_vector_t& q, const joint_vector_t& dq, model_terms_t& terms)
{
    if (!q.allFinite() || !dq.allFinite())
        throw std::invalid_argument("OpenArmModel: non-finite joint state");
    terms.M.setZero();  // zero blocks between the two arms
    terms.C.setZero();
    terms.G.setZero();
    if (hasLeft_)
        addArm(leftModel_, leftIndex_, q, dq, terms);
    if (hasRight_)
        addArm(rightModel_, rightIndex_, q, dq, terms);
}

template <size_t NJOINTS>
typename OpenArmModel<NJOINTS>::model_terms_t OpenArmModel<NJOINTS>::computeTerms(const joint_vector_t& q,
    const joint_vector_t& dq)
{
    model_terms_t terms;
    computeTerms(q, dq, terms);
    return terms;
}

template <size_t NJOINTS>
const std::vector<std::string>& OpenArmModel<NJOINTS>::jointNames() const
{
    return jointNames_;
}

template <size_t NJOINTS>
typename OpenArmModel<NJOINTS>::joint_vector_t OpenArmModel<NJOINTS>::jointLowerLimit() const
{
    joint_vector_t limit;
    for (int j = 0; j < 7; ++j)
    {
        if (hasLeft_)
            limit(leftIndex_[j]) = OpenArm::jointLowerLimit()(j);
        if (hasRight_)
            limit(rightIndex_[j]) = OpenArm::jointLowerLimit()(7 + j);
    }
    return limit;
}

template <size_t NJOINTS>
typename OpenArmModel<NJOINTS>::joint_vector_t OpenArmModel<NJOINTS>::jointUpperLimit() const
{
    joint_vector_t limit;
    for (int j = 0; j < 7; ++j)
    {
        if (hasLeft_)
            limit(leftIndex_[j]) = OpenArm::jointUpperLimit()(j);
        if (hasRight_)
            limit(rightIndex_[j]) = OpenArm::jointUpperLimit()(7 + j);
    }
    return limit;
}

template <size_t NJOINTS>
OpenArmSystem<NJOINTS>::OpenArmSystem() : ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>(), model_()
{
}

template <size_t NJOINTS>
OpenArmSystem<NJOINTS>::OpenArmSystem(const std::vector<std::string>& jointNames)
    : ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>(), model_(jointNames)
{
}

template <size_t NJOINTS>
OpenArmSystem<NJOINTS>::OpenArmSystem(const OpenArmSystem& arg)
    : ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>(arg), model_(arg.model_)
{
}

template <size_t NJOINTS>
OpenArmSystem<NJOINTS>::~OpenArmSystem()
{
}

template <size_t NJOINTS>
OpenArmSystem<NJOINTS>* OpenArmSystem<NJOINTS>::clone() const
{
    return new OpenArmSystem<NJOINTS>(*this);
}

template <size_t NJOINTS>
void OpenArmSystem<NJOINTS>::computeControlledDynamics(const ct::core::StateVector<STATE_DIM>& state,
    const ct::core::Time& t,
    const ct::core::ControlVector<CONTROL_DIM>& control,
    ct::core::StateVector<STATE_DIM>& derivative)
{
    const Eigen::Matrix<double, NJOINTS, 1> q = state.template head<NJOINTS>();
    const Eigen::Matrix<double, NJOINTS, 1> dq = state.template tail<NJOINTS>();
    const ModelTerms<NJOINTS> terms = model_.computeTerms(q, dq);
    derivative.template head<NJOINTS>() = dq;
    derivative.template tail<NJOINTS>() =
        terms.M.ldlt().solve(Eigen::Matrix<double, NJOINTS, 1>(control) - terms.C * dq - terms.G);
}

template <size_t NJOINTS>
OpenArmModel<NJOINTS>& OpenArmSystem<NJOINTS>::model()
{
    return model_;
}

}  // namespace OpenArm
}  // namespace models
}  // namespace ct
