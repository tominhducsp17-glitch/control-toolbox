/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <stdexcept>

namespace ct {
namespace nlc {

template <size_t NJOINTS, class MODEL>
ComputedTorqueController<NJOINTS, MODEL>::ComputedTorqueController(const MODEL& model)
    : ComputedTorqueController(model, joint_vector_t::Zero(), joint_vector_t::Zero(), ModelEvaluation::REFERENCE)
{
}

template <size_t NJOINTS, class MODEL>
ComputedTorqueController<NJOINTS, MODEL>::ComputedTorqueController(const MODEL& model,
    const joint_vector_t& Kp,
    const joint_vector_t& Kd,
    ModelEvaluation evaluation)
    : Base(model), evaluation_(evaluation)
{
    setGains(Kp, Kd);
}

template <size_t NJOINTS, class MODEL>
ComputedTorqueController<NJOINTS, MODEL>::ComputedTorqueController(const ComputedTorqueController& arg)
    : Base(arg), Kp_(arg.Kp_), Kd_(arg.Kd_), evaluation_(arg.evaluation_)
{
}

template <size_t NJOINTS, class MODEL>
ComputedTorqueController<NJOINTS, MODEL>::~ComputedTorqueController()
{
}

template <size_t NJOINTS, class MODEL>
ComputedTorqueController<NJOINTS, MODEL>* ComputedTorqueController<NJOINTS, MODEL>::clone() const
{
    return new ComputedTorqueController<NJOINTS, MODEL>(*this);
}

template <size_t NJOINTS, class MODEL>
void ComputedTorqueController<NJOINTS, MODEL>::computeControl(const state_vector_t& state,
    const ct::core::Time& t,
    control_vector_t& control)
{
    const joint_vector_t q = state.template head<NJOINTS>();
    const joint_vector_t dq = state.template tail<NJOINTS>();

    // model terms at the chosen state
    const bool atReference = (evaluation_ == ModelEvaluation::REFERENCE);
    const joint_vector_t& qModel = atReference ? this->q_d_ : q;
    const joint_vector_t& dqModel = atReference ? this->dq_d_ : dq;
    this->model_->computeTerms(qModel, dqModel, this->terms_);

    // commanded acceleration: reference acceleration plus PD on the tracking error
    const joint_vector_t ddq_cmd = this->ddq_d_ + Kp_.cwiseProduct(this->q_d_ - q) + Kd_.cwiseProduct(this->dq_d_ - dq);

    control = this->terms_.M * ddq_cmd + this->terms_.C * dqModel + this->terms_.G;
    this->saturate(control);
}

template <size_t NJOINTS, class MODEL>
void ComputedTorqueController<NJOINTS, MODEL>::setGains(const joint_vector_t& Kp, const joint_vector_t& Kd)
{
    if (!Kp.allFinite() || !Kd.allFinite() || (Kp.array() < 0.0).any() || (Kd.array() < 0.0).any())
        throw std::invalid_argument("ComputedTorqueController: gains must be finite and >= 0");
    Kp_ = Kp;
    Kd_ = Kd;
}

template <size_t NJOINTS, class MODEL>
auto ComputedTorqueController<NJOINTS, MODEL>::getKp() const -> const joint_vector_t&
{
    return Kp_;
}

template <size_t NJOINTS, class MODEL>
auto ComputedTorqueController<NJOINTS, MODEL>::getKd() const -> const joint_vector_t&
{
    return Kd_;
}

template <size_t NJOINTS, class MODEL>
ModelEvaluation ComputedTorqueController<NJOINTS, MODEL>::getModelEvaluation() const
{
    return evaluation_;
}

}  // namespace nlc
}  // namespace ct
