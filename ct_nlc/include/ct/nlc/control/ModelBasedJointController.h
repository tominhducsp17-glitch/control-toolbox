/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <memory>

#include <ct/core/core.h>

namespace ct {
namespace nlc {

//! the state at which a controller evaluates the model terms M, C, G
enum class ModelEvaluation
{
    REFERENCE,  //!< at the reference (q_d, dq_d): feedforward, independent of the measurement noise
    MEASURED    //!< at the measured state (q, dq): classical feedback linearization
};

/*!
 * \brief Base class of the joint-space controllers that use a robot model
 *
 * The controller holds its own model (cloned with the controller). Every controller has the same input:
 *  - the state x = [q; dq] and the time t, passed to computeControl() like any ct::core::Controller,
 *  - the reference (q_d, dq_d, ddq_d), set with setReference(),
 *  - the model terms M, C, G, computed from the model inside computeControl().
 * The output is the joint torque u [N m], limited to |u_i| <= torqueLimit_i.
 *
 * A new controller derives from this class and overrides computeControl() and clone().
 *
 * \tparam NJOINTS number of joints
 * \tparam MODEL joint-space model with typedefs joint_vector_t and model_terms_t (members M, C, G),
 *         computeTerms(q, dq, terms) and clone(), e.g. ct::models::OpenArm::OpenArmModel<NJOINTS>
 */
template <size_t NJOINTS, class MODEL>
class ModelBasedJointController : public ct::core::Controller<2 * NJOINTS, NJOINTS>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM = 2 * NJOINTS;
    static const size_t CONTROL_DIM = NJOINTS;

    typedef MODEL model_t;
    typedef typename MODEL::model_terms_t model_terms_t;
    typedef Eigen::Matrix<double, NJOINTS, 1> joint_vector_t;
    typedef ct::core::StateVector<STATE_DIM> state_vector_t;
    typedef ct::core::ControlVector<CONTROL_DIM> control_vector_t;

    explicit ModelBasedJointController(const MODEL& model);
    ModelBasedJointController(const ModelBasedJointController& arg);
    virtual ~ModelBasedJointController();

    virtual ModelBasedJointController* clone() const override = 0;

    //! reference position, velocity and acceleration; throws std::invalid_argument for non-finite values
    void setReference(const joint_vector_t& q_d, const joint_vector_t& dq_d, const joint_vector_t& ddq_d);
    const joint_vector_t& getReferencePosition() const;
    const joint_vector_t& getReferenceVelocity() const;
    const joint_vector_t& getReferenceAcceleration() const;

    //! |u_i| <= limit_i (infinity: no limit); throws std::invalid_argument for a limit <= 0
    void setTorqueLimit(const joint_vector_t& limit);
    const joint_vector_t& getTorqueLimit() const;
    //! true if the torque limit changed the last control
    bool lastControlSaturated() const;

    MODEL& model();

protected:
    //! clamps the control to the torque limit, records whether it did; throws std::runtime_error if not finite
    void saturate(control_vector_t& control);

    std::shared_ptr<MODEL> model_;
    model_terms_t terms_;  //!< M, C, G of the last evaluation
    joint_vector_t q_d_;
    joint_vector_t dq_d_;
    joint_vector_t ddq_d_;
    joint_vector_t torqueLimit_;
    bool saturated_;
};

}  // namespace nlc
}  // namespace ct
