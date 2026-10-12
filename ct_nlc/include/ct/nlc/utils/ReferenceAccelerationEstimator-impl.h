/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <stdexcept>

namespace ct {
namespace nlc {

template <size_t NJOINTS>
ReferenceAccelerationEstimator<NJOINTS>::ReferenceAccelerationEstimator(double cutoffHz, double maxAbsAcceleration)
    : cutoffHz_(cutoffHz),
      maxAbsAcceleration_(maxAbsAcceleration),
      hasPrevious_(false),
      tPrevious_(0.0),
      dqPrevious_(joint_vector_t::Zero()),
      estimate_(joint_vector_t::Zero())
{
    if (!(cutoffHz > 0.0) || !(maxAbsAcceleration > 0.0))
        throw std::invalid_argument("ReferenceAccelerationEstimator: cut-off and acceleration limit must be > 0");
}

template <size_t NJOINTS>
ReferenceAccelerationEstimator<NJOINTS>::~ReferenceAccelerationEstimator()
{
}

template <size_t NJOINTS>
auto ReferenceAccelerationEstimator<NJOINTS>::update(double t, const joint_vector_t& dq_d) -> const joint_vector_t&
{
    if (hasPrevious_ && t <= tPrevious_)
        return estimate_;
    if (hasPrevious_)
    {
        const double dt = t - tPrevious_;
        const double alpha = 1.0 - std::exp(-2.0 * M_PI * cutoffHz_ * dt);
        estimate_ += alpha * ((dq_d - dqPrevious_) / dt - estimate_);
        estimate_ = estimate_.cwiseMax(-maxAbsAcceleration_).cwiseMin(maxAbsAcceleration_);
    }
    hasPrevious_ = true;
    tPrevious_ = t;
    dqPrevious_ = dq_d;
    return estimate_;
}

template <size_t NJOINTS>
auto ReferenceAccelerationEstimator<NJOINTS>::getEstimate() const -> const joint_vector_t&
{
    return estimate_;
}

template <size_t NJOINTS>
void ReferenceAccelerationEstimator<NJOINTS>::reset()
{
    hasPrevious_ = false;
    estimate_.setZero();
}

}  // namespace nlc
}  // namespace ct
