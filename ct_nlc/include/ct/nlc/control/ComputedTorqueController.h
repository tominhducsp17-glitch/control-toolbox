/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "ModelBasedJointController.h"

namespace ct {
namespace nlc {

/*!
 * \brief Computed-torque control (feedback linearization) in joint space
 *
 *   u = M(q*) (ddq_d + Kp (q_d - q) + Kd (dq_d - dq)) + C(q*, dq*) dq* + G(q*)
 *
 * with (q*, dq*) the reference (ModelEvaluation::REFERENCE) or the measured state (ModelEvaluation::MEASURED).
 * Kp, Kd are diagonal gains [1/s^2], [1/s].
 *
 * With Kp = Kd = 0 and REFERENCE it is the inverse dynamics of the reference,
 *   u = M(q_d) ddq_d + C(q_d, dq_d) dq_d + G(q_d),
 * the feedforward for a motor driver that closes the PD loop itself.
 */
template <size_t NJOINTS, class MODEL>
class ComputedTorqueController : public ModelBasedJointController<NJOINTS, MODEL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ModelBasedJointController<NJOINTS, MODEL> Base;
    typedef typename Base::joint_vector_t joint_vector_t;
    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;

    //! inverse dynamics of the reference (feedforward): Kp = Kd = 0, ModelEvaluation::REFERENCE
    explicit ComputedTorqueController(const MODEL& model);
    ComputedTorqueController(const MODEL& model,
        const joint_vector_t& Kp,
        const joint_vector_t& Kd,
        ModelEvaluation evaluation = ModelEvaluation::MEASURED);
    ComputedTorqueController(const ComputedTorqueController& arg);
    virtual ~ComputedTorqueController();

    virtual ComputedTorqueController* clone() const override;

    virtual void computeControl(const state_vector_t& state,
        const ct::core::Time& t,
        control_vector_t& control) override;

    //! throws std::invalid_argument for negative or non-finite gains
    void setGains(const joint_vector_t& Kp, const joint_vector_t& Kd);
    const joint_vector_t& getKp() const;
    const joint_vector_t& getKd() const;
    ModelEvaluation getModelEvaluation() const;

private:
    joint_vector_t Kp_;
    joint_vector_t Kd_;
    ModelEvaluation evaluation_;
};

}  // namespace nlc
}  // namespace ct
