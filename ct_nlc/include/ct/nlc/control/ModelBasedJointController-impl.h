/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <limits>
#include <stdexcept>

namespace ct {
namespace nlc {

template <size_t NJOINTS, class MODEL>
ModelBasedJointController<NJOINTS, MODEL>::ModelBasedJointController(const MODEL& model)
    : model_(model.clone()),
      q_d_(joint_vector_t::Zero()),
      dq_d_(joint_vector_t::Zero()),
      ddq_d_(joint_vector_t::Zero()),
      torqueLimit_(joint_vector_t::Constant(std::numeric_limits<double>::infinity())),
      saturated_(false)
{
}

template <size_t NJOINTS, class MODEL>
ModelBasedJointController<NJOINTS, MODEL>::ModelBasedJointController(const ModelBasedJointController& arg)
    : ct::core::Controller<2 * NJOINTS, NJOINTS>(arg),
      model_(arg.model_->clone()),
      terms_(arg.terms_),
      q_d_(arg.q_d_),
      dq_d_(arg.dq_d_),
      ddq_d_(arg.ddq_d_),
      torqueLimit_(arg.torqueLimit_),
      saturated_(arg.saturated_)
{
}

template <size_t NJOINTS, class MODEL>
ModelBasedJointController<NJOINTS, MODEL>::~ModelBasedJointController()
{
}

template <size_t NJOINTS, class MODEL>
void ModelBasedJointController<NJOINTS, MODEL>::setReference(const joint_vector_t& q_d,
    const joint_vector_t& dq_d,
    const joint_vector_t& ddq_d)
{
    if (!q_d.allFinite() || !dq_d.allFinite() || !ddq_d.allFinite())
        throw std::invalid_argument("ModelBasedJointController: reference must be finite");
    q_d_ = q_d;
    dq_d_ = dq_d;
    ddq_d_ = ddq_d;
}

template <size_t NJOINTS, class MODEL>
auto ModelBasedJointController<NJOINTS, MODEL>::getReferencePosition() const -> const joint_vector_t&
{
    return q_d_;
}

template <size_t NJOINTS, class MODEL>
auto ModelBasedJointController<NJOINTS, MODEL>::getReferenceVelocity() const -> const joint_vector_t&
{
    return dq_d_;
}

template <size_t NJOINTS, class MODEL>
auto ModelBasedJointController<NJOINTS, MODEL>::getReferenceAcceleration() const -> const joint_vector_t&
{
    return ddq_d_;
}

template <size_t NJOINTS, class MODEL>
void ModelBasedJointController<NJOINTS, MODEL>::setTorqueLimit(const joint_vector_t& limit)
{
    if (!(limit.array() > 0.0).all())
        throw std::invalid_argument("ModelBasedJointController: torque limit must be > 0");
    torqueLimit_ = limit;
}

template <size_t NJOINTS, class MODEL>
auto ModelBasedJointController<NJOINTS, MODEL>::getTorqueLimit() const -> const joint_vector_t&
{
    return torqueLimit_;
}

template <size_t NJOINTS, class MODEL>
bool ModelBasedJointController<NJOINTS, MODEL>::lastControlSaturated() const
{
    return saturated_;
}

template <size_t NJOINTS, class MODEL>
MODEL& ModelBasedJointController<NJOINTS, MODEL>::model()
{
    return *model_;
}

template <size_t NJOINTS, class MODEL>
void ModelBasedJointController<NJOINTS, MODEL>::saturate(control_vector_t& control)
{
    if (!control.allFinite())
        throw std::runtime_error("ModelBasedJointController: non-finite control");
    const control_vector_t unclamped = control;
    control = control.cwiseMax(-torqueLimit_).cwiseMin(torqueLimit_);
    saturated_ = (control - unclamped).cwiseAbs().maxCoeff() > 0.0;
}

}  // namespace nlc
}  // namespace ct
