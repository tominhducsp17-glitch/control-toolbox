/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Core>

namespace ct {
namespace nlc {

/*!
 * \brief Reference acceleration ddq_d from a stream of reference velocities dq_d
 *
 * Finite difference, first-order low-pass with cut-off frequency cutoffHz, clamp to |ddq_d_i| <= maxAbsAcceleration.
 * For a reference that only gives q_d and dq_d (e.g. a stream of joint commands).
 */
template <size_t NJOINTS>
class ReferenceAccelerationEstimator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, NJOINTS, 1> joint_vector_t;

    //! throws std::invalid_argument unless both values are > 0
    ReferenceAccelerationEstimator(double cutoffHz, double maxAbsAcceleration);
    virtual ~ReferenceAccelerationEstimator();

    //! adds the sample dq_d at time t [s] and returns the estimate; samples with t <= last t are ignored
    const joint_vector_t& update(double t, const joint_vector_t& dq_d);
    const joint_vector_t& getEstimate() const;
    void reset();

private:
    double cutoffHz_;
    double maxAbsAcceleration_;
    bool hasPrevious_;
    double tPrevious_;
    joint_vector_t dqPrevious_;
    joint_vector_t estimate_;
};

}  // namespace nlc
}  // namespace ct
