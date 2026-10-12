/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace nlc {

template <size_t NJOINTS, class MODEL>
GravityCompensationController<NJOINTS, MODEL>::GravityCompensationController(const MODEL& model,
    ModelEvaluation evaluation)
    : Base(model), evaluation_(evaluation)
{
}

template <size_t NJOINTS, class MODEL>
GravityCompensationController<NJOINTS, MODEL>::GravityCompensationController(const GravityCompensationController& arg)
    : Base(arg), evaluation_(arg.evaluation_)
{
}

template <size_t NJOINTS, class MODEL>
GravityCompensationController<NJOINTS, MODEL>::~GravityCompensationController()
{
}

template <size_t NJOINTS, class MODEL>
GravityCompensationController<NJOINTS, MODEL>* GravityCompensationController<NJOINTS, MODEL>::clone() const
{
    return new GravityCompensationController<NJOINTS, MODEL>(*this);
}

template <size_t NJOINTS, class MODEL>
void GravityCompensationController<NJOINTS, MODEL>::computeControl(const state_vector_t& state,
    const ct::core::Time& t,
    control_vector_t& control)
{
    if (evaluation_ == ModelEvaluation::REFERENCE)
        this->model_->computeTerms(this->q_d_, joint_vector_t::Zero(), this->terms_);
    else
        this->model_->computeTerms(state.template head<NJOINTS>(), joint_vector_t::Zero(), this->terms_);

    control = this->terms_.G;
    this->saturate(control);
}

template <size_t NJOINTS, class MODEL>
ModelEvaluation GravityCompensationController<NJOINTS, MODEL>::getModelEvaluation() const
{
    return evaluation_;
}

}  // namespace nlc
}  // namespace ct
