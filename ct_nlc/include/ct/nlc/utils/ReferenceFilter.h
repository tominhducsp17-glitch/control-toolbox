/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Core>

namespace ct {
namespace nlc {

/*!
 * \brief Smooth reference (q_d, dq_d, ddq_d) at the control rate from a slower stream of target positions u
 *
 * Critically damped 4th-order filter per joint,
 *
 *   q_d / u = wn^4 / (s + wn)^4,
 *   q_d'''' = wn^4 (u - q_d) - 4 wn^3 dq_d - 6 wn^2 ddq_d - 4 wn q_d''',
 *
 * integrated with semi-implicit Euler once per control cycle (u is held between its samples). The output is
 * continuous up to the jerk, without overshoot, and lags u by about 4 / wn. Feeding u at t + 4 / wn (targets known
 * ahead, e.g. a trajectory file or an action chunk) cancels most of the lag.
 * Keep wn * dt <= ~0.1 (e.g. wn <= 100 rad/s at 1 kHz).
 */
template <size_t NJOINTS>
class ReferenceFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, NJOINTS, 1> joint_vector_t;

    //! throws std::invalid_argument unless wn > 0 [rad/s]
    explicit ReferenceFilter(double wn);
    ReferenceFilter(const ReferenceFilter& arg);
    virtual ~ReferenceFilter();

    //! at rest at q (q_d = q, all derivatives 0)
    void reset(const joint_vector_t& q);
    //! one step of length dt [s] towards the target position u; throws std::invalid_argument for non-finite input
    void update(const joint_vector_t& u, double dt);

    const joint_vector_t& getPosition() const;
    const joint_vector_t& getVelocity() const;
    const joint_vector_t& getAcceleration() const;
    double getNaturalFrequency() const;

private:
    double wn_;
    joint_vector_t q_;
    joint_vector_t dq_;
    joint_vector_t ddq_;
    joint_vector_t dddq_;
};

}  // namespace nlc
}  // namespace ct
