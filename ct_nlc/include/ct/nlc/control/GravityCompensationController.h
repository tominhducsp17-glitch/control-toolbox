/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "ModelBasedJointController.h"

namespace ct {
namespace nlc {

/*!
 * \brief Gravity compensation: u = G(q_d) (ModelEvaluation::REFERENCE) or u = G(q) (ModelEvaluation::MEASURED)
 */
template <size_t NJOINTS, class MODEL>
class GravityCompensationController : public ModelBasedJointController<NJOINTS, MODEL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ModelBasedJointController<NJOINTS, MODEL> Base;
    typedef typename Base::joint_vector_t joint_vector_t;
    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;

    explicit GravityCompensationController(const MODEL& model, ModelEvaluation evaluation = ModelEvaluation::REFERENCE);
    GravityCompensationController(const GravityCompensationController& arg);
    virtual ~GravityCompensationController();

    virtual GravityCompensationController* clone() const override;

    virtual void computeControl(const state_vector_t& state,
        const ct::core::Time& t,
        control_vector_t& control) override;

    ModelEvaluation getModelEvaluation() const;

private:
    ModelEvaluation evaluation_;
};

}  // namespace nlc
}  // namespace ct
