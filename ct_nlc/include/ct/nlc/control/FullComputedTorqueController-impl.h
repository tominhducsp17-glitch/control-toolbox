/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <stdexcept>

namespace ct {
namespace nlc {

template <size_t NJOINTS, class MODEL>
FullComputedTorqueController<NJOINTS, MODEL>::FullComputedTorqueController(const MODEL& model,
    const joint_vector_t& Kp,
    const joint_vector_t& Kd)
    : Base(model),
      bias_(joint_vector_t::Zero()),
      fc_(joint_vector_t::Zero()),
      fv_(joint_vector_t::Zero()),
      v_(0.02),
      Ki_(joint_vector_t::Zero()),
      integralLimit_(joint_vector_t::Zero()),
      integral_(joint_vector_t::Zero()),
      integratorActive_(false),
      hasPreviousTime_(false),
      tPrevious_(0.0)
{
    setGains(Kp, Kd);
}

template <size_t NJOINTS, class MODEL>
FullComputedTorqueController<NJOINTS, MODEL>::FullComputedTorqueController(const FullComputedTorqueController& arg)
    : Base(arg),
      Kp_(arg.Kp_),
      Kd_(arg.Kd_),
      bias_(arg.bias_),
      fc_(arg.fc_),
      fv_(arg.fv_),
      v_(arg.v_),
      Ki_(arg.Ki_),
      integralLimit_(arg.integralLimit_),
      integral_(arg.integral_),
      integratorActive_(arg.integratorActive_),
      hasPreviousTime_(arg.hasPreviousTime_),
      tPrevious_(arg.tPrevious_)
{
}

template <size_t NJOINTS, class MODEL>
FullComputedTorqueController<NJOINTS, MODEL>::~FullComputedTorqueController()
{
}

template <size_t NJOINTS, class MODEL>
FullComputedTorqueController<NJOINTS, MODEL>* FullComputedTorqueController<NJOINTS, MODEL>::clone() const
{
    return new FullComputedTorqueController<NJOINTS, MODEL>(*this);
}

template <size_t NJOINTS, class MODEL>
void FullComputedTorqueController<NJOINTS, MODEL>::computeControl(const state_vector_t& state,
    const ct::core::Time& t,
    control_vector_t& control)
{
    const joint_vector_t q = state.template head<NJOINTS>();
    const joint_vector_t dq = state.template tail<NJOINTS>();
    const joint_vector_t e = this->q_d_ - q;

    if (integratorActive_ && hasPreviousTime_)
    {
        const double dt = t - tPrevious_;
        if (dt > 0.0 && dt <= 0.1)
        {
            integral_ += dt * Ki_.cwiseProduct(e);
            integral_ = integral_.cwiseMax(-integralLimit_).cwiseMin(integralLimit_);
        }
    }
    hasPreviousTime_ = true;
    tPrevious_ = t;

    this->model_->computeTerms(q, dq, this->terms_);
    control = this->terms_.M * this->ddq_d_ + this->terms_.C * dq + this->terms_.G +
              feedforwardCorrection(this->dq_d_) + Kp_.cwiseProduct(e) + Kd_.cwiseProduct(this->dq_d_ - dq) + integral_;
    this->saturate(control);
}

template <size_t NJOINTS, class MODEL>
void FullComputedTorqueController<NJOINTS, MODEL>::setGains(const joint_vector_t& Kp, const joint_vector_t& Kd)
{
    if (!(Kp.allFinite() && (Kp.array() >= 0.0).all()) || !(Kd.allFinite() && (Kd.array() >= 0.0).all()))
        throw std::invalid_argument("FullComputedTorqueController: gains must be finite and >= 0");
    Kp_ = Kp;
    Kd_ = Kd;
}

template <size_t NJOINTS, class MODEL>
void FullComputedTorqueController<NJOINTS, MODEL>::setModelCorrection(const joint_vector_t& bias)
{
    if (!bias.allFinite())
        throw std::invalid_argument("FullComputedTorqueController: model correction must be finite");
    bias_ = bias;
}

template <size_t NJOINTS, class MODEL>
void FullComputedTorqueController<NJOINTS, MODEL>::setFrictionFeedforward(const joint_vector_t& fc,
    const joint_vector_t& fv,
    double v)
{
    if (!(fc.allFinite() && (fc.array() >= 0.0).all()) || !(fv.allFinite() && (fv.array() >= 0.0).all()) ||
        !(v > 0.0) || !std::isfinite(v))
        throw std::invalid_argument("FullComputedTorqueController: friction must be finite, >= 0, v > 0");
    fc_ = fc;
    fv_ = fv;
    v_ = v;
}

template <size_t NJOINTS, class MODEL>
void FullComputedTorqueController<NJOINTS, MODEL>::setIntegralGain(const joint_vector_t& Ki,
    const joint_vector_t& integralLimit)
{
    if (!(Ki.allFinite() && (Ki.array() >= 0.0).all()) ||
        !(integralLimit.allFinite() && (integralLimit.array() >= 0.0).all()))
        throw std::invalid_argument("FullComputedTorqueController: integral gain and limit must be finite and >= 0");
    Ki_ = Ki;
    integralLimit_ = integralLimit;
    integral_ = integral_.cwiseMax(-integralLimit_).cwiseMin(integralLimit_);
}

template <size_t NJOINTS, class MODEL>
void FullComputedTorqueController<NJOINTS, MODEL>::setIntegratorActive(bool active)
{
    integratorActive_ = active;
}

template <size_t NJOINTS, class MODEL>
void FullComputedTorqueController<NJOINTS, MODEL>::resetIntegral()
{
    integral_.setZero();
    hasPreviousTime_ = false;
}

template <size_t NJOINTS, class MODEL>
auto FullComputedTorqueController<NJOINTS, MODEL>::getIntegral() const -> const joint_vector_t&
{
    return integral_;
}

template <size_t NJOINTS, class MODEL>
auto FullComputedTorqueController<NJOINTS, MODEL>::feedforwardCorrection(const joint_vector_t& dq_d) const
    -> joint_vector_t
{
    return bias_ + fc_.cwiseProduct((dq_d / v_).array().tanh().matrix()) + fv_.cwiseProduct(dq_d);
}

template <size_t NJOINTS, class MODEL>
auto FullComputedTorqueController<NJOINTS, MODEL>::getKp() const -> const joint_vector_t&
{
    return Kp_;
}

template <size_t NJOINTS, class MODEL>
auto FullComputedTorqueController<NJOINTS, MODEL>::getKd() const -> const joint_vector_t&
{
    return Kd_;
}

}  // namespace nlc
}  // namespace ct
