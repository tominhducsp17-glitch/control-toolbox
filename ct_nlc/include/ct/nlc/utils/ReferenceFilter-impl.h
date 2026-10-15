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
ReferenceFilter<NJOINTS>::ReferenceFilter(double wn)
    : wn_(wn),
      q_(joint_vector_t::Zero()),
      dq_(joint_vector_t::Zero()),
      ddq_(joint_vector_t::Zero()),
      dddq_(joint_vector_t::Zero())
{
    if (!(wn > 0.0) || !std::isfinite(wn))
        throw std::invalid_argument("ReferenceFilter: wn must be > 0");
}

template <size_t NJOINTS>
ReferenceFilter<NJOINTS>::ReferenceFilter(const ReferenceFilter& arg)
    : wn_(arg.wn_), q_(arg.q_), dq_(arg.dq_), ddq_(arg.ddq_), dddq_(arg.dddq_)
{
}

template <size_t NJOINTS>
ReferenceFilter<NJOINTS>::~ReferenceFilter()
{
}

template <size_t NJOINTS>
void ReferenceFilter<NJOINTS>::reset(const joint_vector_t& q)
{
    if (!q.allFinite())
        throw std::invalid_argument("ReferenceFilter: position must be finite");
    q_ = q;
    dq_.setZero();
    ddq_.setZero();
    dddq_.setZero();
}

template <size_t NJOINTS>
void ReferenceFilter<NJOINTS>::update(const joint_vector_t& u, double dt)
{
    if (!u.allFinite() || !(dt > 0.0))
        throw std::invalid_argument("ReferenceFilter: target must be finite and dt > 0");
    const double w2 = wn_ * wn_;
    const double w3 = w2 * wn_;
    const double w4 = w3 * wn_;
    const joint_vector_t d4q = w4 * (u - q_) - 4.0 * w3 * dq_ - 6.0 * w2 * ddq_ - 4.0 * wn_ * dddq_;
    // semi-implicit Euler: each derivative uses the one just updated
    dddq_ += dt * d4q;
    ddq_ += dt * dddq_;
    dq_ += dt * ddq_;
    q_ += dt * dq_;
}

template <size_t NJOINTS>
auto ReferenceFilter<NJOINTS>::getPosition() const -> const joint_vector_t&
{
    return q_;
}

template <size_t NJOINTS>
auto ReferenceFilter<NJOINTS>::getVelocity() const -> const joint_vector_t&
{
    return dq_;
}

template <size_t NJOINTS>
auto ReferenceFilter<NJOINTS>::getAcceleration() const -> const joint_vector_t&
{
    return ddq_;
}

template <size_t NJOINTS>
double ReferenceFilter<NJOINTS>::getNaturalFrequency() const
{
    return wn_;
}

}  // namespace nlc
}  // namespace ct
