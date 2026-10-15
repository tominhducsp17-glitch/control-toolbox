/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "ModelBasedJointController.h"

namespace ct {
namespace nlc {

/*!
 * \brief Full computed-torque control of a torque-commanded arm (motor driver gains kp = kd = 0)
 *
 * The whole joint torque comes from this controller, with the feedback gains in torque units:
 *
 *   u = M(q) ddq_d + C(q, dq) dq + G(q)               model at the measured state
 *     + b                                             measured model correction (constant per joint)
 *     + fc tanh(dq_d / v) + fv dq_d                   friction feedforward
 *     + Kp (q_d - q) + Kd (dq_d - dq)                 PD, Kp [N m/rad], Kd [N m s/rad]
 *     + Ki int (q_d - q) dt,  |.| <= integralLimit    integral (only while active, see setIntegratorActive())
 *
 * The integral uses the time t of computeControl(): the step is t - t_previous (steps <= 0 or > 0.1 s add nothing).
 * Unlike ComputedTorqueController the gains are not multiplied by M: on the OpenArm the wrist inertia is three
 * orders of magnitude below the shoulder's, and per-joint torque gains could be tuned on the robot one joint at a
 * time (configuration and results in ros/openarm_controller/config/controller.yaml).
 */
template <size_t NJOINTS, class MODEL>
class FullComputedTorqueController : public ModelBasedJointController<NJOINTS, MODEL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ModelBasedJointController<NJOINTS, MODEL> Base;
    typedef typename Base::joint_vector_t joint_vector_t;
    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;

    //! PD only (no correction, friction or integral); throws std::invalid_argument for negative or non-finite gains
    FullComputedTorqueController(const MODEL& model, const joint_vector_t& Kp, const joint_vector_t& Kd);
    FullComputedTorqueController(const FullComputedTorqueController& arg);
    virtual ~FullComputedTorqueController();

    virtual FullComputedTorqueController* clone() const override;

    virtual void computeControl(const state_vector_t& state,
        const ct::core::Time& t,
        control_vector_t& control) override;

    //! the setters throw std::invalid_argument for negative (bias: non-finite) values, v must be > 0
    void setGains(const joint_vector_t& Kp, const joint_vector_t& Kd);
    void setModelCorrection(const joint_vector_t& bias);
    void setFrictionFeedforward(const joint_vector_t& fc, const joint_vector_t& fv, double v);
    void setIntegralGain(const joint_vector_t& Ki, const joint_vector_t& integralLimit);

    //! the integral accumulates only while active; when inactive it keeps its value (still added to u)
    void setIntegratorActive(bool active);
    void resetIntegral();
    const joint_vector_t& getIntegral() const;

    //! b + fc tanh(dq_d / v) + fv dq_d: the model-independent feedforward, also usable with a driver PD
    joint_vector_t feedforwardCorrection(const joint_vector_t& dq_d) const;

    const joint_vector_t& getKp() const;
    const joint_vector_t& getKd() const;

private:
    joint_vector_t Kp_;
    joint_vector_t Kd_;
    joint_vector_t bias_;
    joint_vector_t fc_;
    joint_vector_t fv_;
    double v_;
    joint_vector_t Ki_;
    joint_vector_t integralLimit_;
    joint_vector_t integral_;
    bool integratorActive_;
    bool hasPreviousTime_;
    double tPrevious_;
};

}  // namespace nlc
}  // namespace ct
