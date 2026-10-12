/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>

#include <ct/models/OpenArm/codegen/OpenArmLeftModel.h>
#include <ct/models/OpenArm/codegen/OpenArmRightModel.h>
#include <ct/models/OpenArm/codegen/OpenArmJointLimits.h>
#include <ct/models/OpenArm/OpenArmUrdfNames.h>

#include <array>
#include <string>
#include <vector>

namespace ct {
namespace models {
namespace OpenArm {

//! joint-space model terms at one state: M(q) ddq + C(q, dq) dq + G(q) = tau
template <size_t NJOINTS>
struct ModelTerms
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, NJOINTS, NJOINTS> M;  //!< joint-space inertia matrix
    Eigen::Matrix<double, NJOINTS, NJOINTS> C;  //!< Coriolis matrix
    Eigen::Matrix<double, NJOINTS, 1> G;        //!< gravity torque
};

/*!
 * \brief OpenArm v1 joint-space model evaluated with generated code (codegen/, written by OpenArmModelCodegen)
 *
 * NJOINTS is 7 (one arm) or 14 (both arms). The joint order is given by joint names: each arm that appears must
 * appear with all its joints J1..J7, in any order. The fingers are fixed at 0 (closed). The two arms hang from the
 * fixed body, so the two-arm model is block diagonal.
 */
template <size_t NJOINTS>
class OpenArmModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, NJOINTS, 1> joint_vector_t;
    typedef ModelTerms<NJOINTS> model_terms_t;

    //! default joints: urdfJointNames() for 14 joints, the right arm for 7 joints
    OpenArmModel();
    //! throws std::invalid_argument for an invalid joint list
    explicit OpenArmModel(const std::vector<std::string>& jointNames);
    OpenArmModel(const OpenArmModel& arg);
    virtual ~OpenArmModel();
    OpenArmModel* clone() const;

    //! M, C, G at the joint state (q, dq); throws std::invalid_argument for non-finite input
    void computeTerms(const joint_vector_t& q, const joint_vector_t& dq, model_terms_t& terms);
    model_terms_t computeTerms(const joint_vector_t& q, const joint_vector_t& dq);

    const std::vector<std::string>& jointNames() const;
    joint_vector_t jointLowerLimit() const;
    joint_vector_t jointUpperLimit() const;

private:
    template <class ArmModel>
    void addArm(ArmModel& armModel,
        const std::array<int, 7>& index,
        const joint_vector_t& q,
        const joint_vector_t& dq,
        model_terms_t& terms);

    std::vector<std::string> jointNames_;
    bool hasLeft_;
    bool hasRight_;
    std::array<int, 7> leftIndex_;   //!< position of left J1..J7 in the joint vector
    std::array<int, 7> rightIndex_;  //!< position of right J1..J7 in the joint vector
    OpenArmLeftModel leftModel_;
    OpenArmRightModel rightModel_;
    Eigen::VectorXd x_;  //!< generated-code input [q; dq] of one arm
};

/*!
 * \brief OpenArm model as a controlled system, dx = [dq; M^-1 (u - C dq - G)]
 *
 * Usable with ct::core integrators and simulators and with ct_optcon solvers.
 */
template <size_t NJOINTS>
class OpenArmSystem : public ct::core::ControlledSystem<2 * NJOINTS, NJOINTS>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM = 2 * NJOINTS;
    static const size_t CONTROL_DIM = NJOINTS;

    OpenArmSystem();
    explicit OpenArmSystem(const std::vector<std::string>& jointNames);
    OpenArmSystem(const OpenArmSystem& arg);
    virtual ~OpenArmSystem();
    virtual OpenArmSystem* clone() const override;

    virtual void computeControlledDynamics(const ct::core::StateVector<STATE_DIM>& state,
        const ct::core::Time& t,
        const ct::core::ControlVector<CONTROL_DIM>& control,
        ct::core::StateVector<STATE_DIM>& derivative) override;

    OpenArmModel<NJOINTS>& model();

private:
    OpenArmModel<NJOINTS> model_;
};

}  // namespace OpenArm
}  // namespace models
}  // namespace ct
