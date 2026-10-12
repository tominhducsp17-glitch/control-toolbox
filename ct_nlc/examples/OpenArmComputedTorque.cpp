/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * Simulates the OpenArm right arm (ct::models::OpenArm::OpenArmSystem, generated model code) following a sine
 * trajectory on all joints, starting 0.05 rad off the trajectory, with two controllers built on ct_nlc:
 *  1. computed-torque control on the measured state (Kp, Kd in the controller),
 *  2. computed-torque feedforward plus a PD loop as in the motor driver, u = kp (q_d - q) + kd (dq_d - dq) + u_ff.
 * Prints the tracking error |q_d - q| of each joint at t = 0.5 s and the largest one after t = 1 s.
 */

#include <cmath>
#include <iostream>

#include <ct/nlc/nlc.h>
#include <ct/models/OpenArm/OpenArm.h>

using namespace ct::nlc;
using ct::models::OpenArm::OpenArmModel;
using ct::models::OpenArm::OpenArmSystem;

typedef Eigen::Matrix<double, 7, 1> Vector7;
typedef ComputedTorqueController<7, OpenArmModel<7>> CTC;

//! sine on every joint around q0
struct SineTrajectory
{
    SineTrajectory()
    {
        q0 << 0.4, 0.1, 0.0, 0.9, 0.0, -0.2, 0.0;
        amplitude << 0.3, 0.2, 0.3, 0.3, 0.3, 0.2, 0.3;
    }
    Vector7 position(double t) const { return q0 + amplitude * std::sin(w * t); }
    Vector7 velocity(double t) const { return amplitude * w * std::cos(w * t); }
    Vector7 acceleration(double t) const { return -amplitude * w * w * std::sin(w * t); }

    Vector7 q0;
    Vector7 amplitude;
    double w = 2.0 * M_PI * 0.5;  //!< 0.5 Hz
};

//! the motor driver law, u = kp (q_d - q) + kd (dq_d - dq) + u_ff, with u_ff from a computed-torque feedforward
class DriverPDWithFeedforward : public ct::core::Controller<14, 7>
{
public:
    DriverPDWithFeedforward(const CTC& feedforward, double kp, double kd)
        : feedforward_(feedforward.clone()), kp_(kp), kd_(kd)
    {
    }
    DriverPDWithFeedforward(const DriverPDWithFeedforward& arg)
        : ct::core::Controller<14, 7>(arg), feedforward_(arg.feedforward_->clone()), kp_(arg.kp_), kd_(arg.kd_)
    {
    }
    virtual ~DriverPDWithFeedforward() {}
    virtual DriverPDWithFeedforward* clone() const override { return new DriverPDWithFeedforward(*this); }

    virtual void computeControl(const ct::core::StateVector<14>& state,
        const ct::core::Time& t,
        ct::core::ControlVector<7>& control) override
    {
        feedforward_->computeControl(state, t, control);
        control += kp_ * (feedforward_->getReferencePosition() - state.head<7>()) +
                   kd_ * (feedforward_->getReferenceVelocity() - state.tail<7>());
    }

    void setReference(const Vector7& q_d, const Vector7& dq_d, const Vector7& ddq_d)
    {
        feedforward_->setReference(q_d, dq_d, ddq_d);
    }

private:
    std::shared_ptr<CTC> feedforward_;
    double kp_;
    double kd_;
};

//! sets the reference of CONTROLLER from the trajectory at the time of each call (also inside the RK4 steps)
template <class CONTROLLER>
class FollowTrajectory : public ct::core::Controller<14, 7>
{
public:
    FollowTrajectory(const CONTROLLER& controller, const SineTrajectory& trajectory)
        : controller_(controller.clone()), trajectory_(trajectory)
    {
    }
    FollowTrajectory(const FollowTrajectory& arg)
        : ct::core::Controller<14, 7>(arg), controller_(arg.controller_->clone()), trajectory_(arg.trajectory_)
    {
    }
    virtual ~FollowTrajectory() {}
    virtual FollowTrajectory* clone() const override { return new FollowTrajectory(*this); }

    virtual void computeControl(const ct::core::StateVector<14>& state,
        const ct::core::Time& t,
        ct::core::ControlVector<7>& control) override
    {
        controller_->setReference(trajectory_.position(t), trajectory_.velocity(t), trajectory_.acceleration(t));
        controller_->computeControl(state, t, control);
    }

private:
    std::shared_ptr<CONTROLLER> controller_;
    SineTrajectory trajectory_;
};

//! 4 s from 0.05 rad off the trajectory; prints |q_d - q| at 0.5 s and the largest |q_d - q| after 1 s
void simulate(const std::string& name, std::shared_ptr<ct::core::Controller<14, 7>> controller)
{
    const SineTrajectory trajectory;
    auto system = std::make_shared<OpenArmSystem<7>>();
    system->setController(controller);
    ct::core::Integrator<14> integrator(system, ct::core::IntegrationType::RK4);

    const double dt = 0.001;
    ct::core::StateVector<14> x;
    x << trajectory.position(0.0) + Vector7::Constant(0.05), trajectory.velocity(0.0);
    Vector7 errorAt05s, maxErrorAfter1s = Vector7::Zero();
    for (int k = 1; k <= 4000; ++k)
    {
        integrator.integrate_n_steps(x, (k - 1) * dt, 1, dt);
        const Vector7 error = (trajectory.position(k * dt) - x.head<7>()).cwiseAbs();
        if (k == 500)
            errorAt05s = error;
        if (k >= 1000)
            maxErrorAfter1s = maxErrorAfter1s.cwiseMax(error);
    }
    const Eigen::IOFormat row(2, 0, ", ", ", ", "", "", "[", "]");
    std::cout << name << std::endl
              << "  |q_d - q| at t = 0.5 s [rad]:  " << errorAt05s.transpose().format(row) << std::endl
              << "  max |q_d - q| after 1 s [rad]: " << maxErrorAfter1s.transpose().format(row) << std::endl;
}

int main(int argc, char** argv)
{
    const OpenArmModel<7> model;  // right arm, joints J1..J7
    const SineTrajectory trajectory;

    simulate("1. computed-torque control on the measured state (Kp 100, Kd 20)",
        std::make_shared<FollowTrajectory<CTC>>(
            CTC(model, Vector7::Constant(100.0), Vector7::Constant(20.0), ModelEvaluation::MEASURED), trajectory));

    simulate("2. feedforward M(q_d) ddq_d + C(q_d, dq_d) dq_d + G(q_d) + driver PD (kp 35, kd 1.2)",
        std::make_shared<FollowTrajectory<DriverPDWithFeedforward>>(
            DriverPDWithFeedforward(CTC(model), 35.0, 1.2), trajectory));

    return 0;
}
