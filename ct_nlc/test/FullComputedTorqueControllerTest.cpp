/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <ct/nlc/nlc.h>
#include <ct/models/OpenArm/OpenArm.h>

using namespace ct::nlc;
using ct::models::OpenArm::OpenArmModel;
using ct::models::OpenArm::OpenArmSystem;

typedef Eigen::Matrix<double, 7, 1> Vector7;
typedef FullComputedTorqueController<7, OpenArmModel<7>> FullCTC;

namespace {

ct::core::StateVector<14> state(const Vector7& q, const Vector7& dq)
{
    ct::core::StateVector<14> x;
    x << q, dq;
    return x;
}

Vector7 pose()
{
    Vector7 q;
    q << 0.4, 0.25, 0.0, 1.4, 0.0, 0.0, 0.0;
    return q;
}

}  // namespace

TEST(FullComputedTorqueControllerTest, ControlLaw)
{
    const Vector7 Kp = (Vector7() << 100, 70, 60, 60, 15, 22, 25).finished();
    const Vector7 Kd = (Vector7() << 5, 7, 4.8, 4.8, 0.18, 0.42, 0.5).finished();
    const Vector7 bias = (Vector7() << 0.41, 0.29, 0.10, 0.21, 0.06, -0.07, 0.08).finished();
    const Vector7 fc = Vector7::Constant(0.5), fv = Vector7::Constant(0.3);
    OpenArmModel<7> model;
    FullCTC ctc(model, Kp, Kd);
    ctc.setModelCorrection(bias);
    ctc.setFrictionFeedforward(fc, fv, 0.02);

    const Vector7 q = pose(), dq = Vector7::Constant(0.1);
    const Vector7 q_d = q + Vector7::Constant(0.01), dq_d = Vector7::Constant(0.2), ddq_d = Vector7::Constant(0.5);
    ctc.setReference(q_d, dq_d, ddq_d);
    ct::core::ControlVector<7> u;
    ctc.computeControl(state(q, dq), 0.0, u);

    const auto terms = model.computeTerms(q, dq);
    const Vector7 friction = fc.cwiseProduct((dq_d / 0.02).array().tanh().matrix()) + fv.cwiseProduct(dq_d);
    const Vector7 expected = terms.M * ddq_d + terms.C * dq + terms.G + bias + friction + Kp.cwiseProduct(q_d - q) +
                             Kd.cwiseProduct(dq_d - dq);
    EXPECT_LT((u - expected).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((ctc.feedforwardCorrection(dq_d) - (bias + friction)).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(FullComputedTorqueControllerTest, IntegralOnlyWhileActiveAndClamped)
{
    FullCTC ctc(OpenArmModel<7>{}, Vector7::Zero(), Vector7::Zero());
    ctc.setIntegralGain(Vector7::Constant(40.0), Vector7::Constant(1.0));
    const Vector7 q = pose();
    ctc.setReference(q + Vector7::Constant(0.01), Vector7::Zero(), Vector7::Zero());
    ct::core::ControlVector<7> u;

    for (int k = 0; k <= 100; ++k)  // inactive: nothing accumulates
        ctc.computeControl(state(q, Vector7::Zero()), k * 0.001, u);
    EXPECT_EQ(ctc.getIntegral().norm(), 0.0);

    ctc.setIntegratorActive(true);
    for (int k = 101; k <= 200; ++k)  // 0.1 s of e = 0.01 rad: 40 * 0.01 * 0.1 = 0.04 N m
        ctc.computeControl(state(q, Vector7::Zero()), k * 0.001, u);
    EXPECT_NEAR(ctc.getIntegral()(0), 0.04, 1e-9);

    ctc.setIntegratorActive(false);  // kept, still in u
    const ct::core::ControlVector<7> uHeld = u;
    ctc.computeControl(state(q, Vector7::Zero()), 0.5, u);
    EXPECT_NEAR(ctc.getIntegral()(0), 0.04, 1e-9);
    EXPECT_LT((u - uHeld).cwiseAbs().maxCoeff(), 1e-9);

    ctc.setIntegratorActive(true);
    for (int k = 501; k <= 5000; ++k)
        ctc.computeControl(state(q, Vector7::Zero()), k * 0.001, u);
    EXPECT_NEAR(ctc.getIntegral()(0), 1.0, 1e-12);  // clamped
    ctc.resetIntegral();
    EXPECT_EQ(ctc.getIntegral().norm(), 0.0);
}

TEST(FullComputedTorqueControllerTest, CloneAndInvalidInput)
{
    FullCTC ctc(OpenArmModel<7>{}, Vector7::Constant(50.0), Vector7::Constant(2.0));
    ctc.setIntegralGain(Vector7::Constant(10.0), Vector7::Constant(0.5));
    ctc.setIntegratorActive(true);
    ctc.setReference(pose(), Vector7::Zero(), Vector7::Zero());
    ct::core::ControlVector<7> u, uCopy;
    ctc.computeControl(state(pose() - Vector7::Constant(0.02), Vector7::Zero()), 0.0, u);
    ctc.computeControl(state(pose() - Vector7::Constant(0.02), Vector7::Zero()), 0.01, u);

    std::shared_ptr<ct::core::Controller<14, 7>> copy(ctc.clone());  // through the ct::core::Controller interface
    copy->computeControl(state(pose() - Vector7::Constant(0.02), Vector7::Zero()), 0.02, uCopy);
    ctc.computeControl(state(pose() - Vector7::Constant(0.02), Vector7::Zero()), 0.02, u);
    EXPECT_EQ(u, uCopy);  // same integral and time

    EXPECT_THROW(ctc.setGains(-Vector7::Ones(), Vector7::Ones()), std::invalid_argument);
    EXPECT_THROW(ctc.setFrictionFeedforward(Vector7::Ones(), Vector7::Ones(), 0.0), std::invalid_argument);
    EXPECT_THROW(ctc.setIntegralGain(Vector7::Ones(), -Vector7::Ones()), std::invalid_argument);
    Vector7 nan = Vector7::Zero();
    nan(3) = std::nan("");
    EXPECT_THROW(ctc.setModelCorrection(nan), std::invalid_argument);
}

//! full CTC on a reference that comes from a 100 Hz position stream through the ReferenceFilter at 1 kHz
class FilteredStreamCTC : public ct::core::Controller<14, 7>
{
public:
    FilteredStreamCTC(const FullCTC& ctc, double wn) : ctc_(ctc.clone()), filter_(wn) { filter_.reset(target(0.0)); }
    FilteredStreamCTC(const FilteredStreamCTC& arg)
        : ct::core::Controller<14, 7>(arg), ctc_(arg.ctc_->clone()), filter_(arg.filter_)
    {
    }
    virtual ~FilteredStreamCTC() {}
    virtual FilteredStreamCTC* clone() const override { return new FilteredStreamCTC(*this); }

    //! once per 1 ms control cycle: the 100 Hz sample (held) at t + lead into the filter
    void step(double t, double lead)
    {
        filter_.update(target(std::floor((t + lead) * 100.0) / 100.0), 0.001);
        ctc_->setReference(filter_.getPosition(), filter_.getVelocity(), filter_.getAcceleration());
    }

    virtual void computeControl(const ct::core::StateVector<14>& state,
        const ct::core::Time& t,
        ct::core::ControlVector<7>& control) override
    {
        ctc_->computeControl(state, t, control);
    }

    static Vector7 target(double t) { return pose() + Vector7::Constant(0.2 * std::sin(2.0 * M_PI * 0.5 * t)); }
    const ReferenceFilter<7>& filter() const { return filter_; }

private:
    std::shared_ptr<FullCTC> ctc_;
    ReferenceFilter<7> filter_;
};

TEST(FullComputedTorqueControllerTest, TracksAFilteredPositionStreamInSimulation)
{
    auto system = std::make_shared<OpenArmSystem<7>>();
    auto controller = std::make_shared<FilteredStreamCTC>(
        FullCTC(OpenArmModel<7>{}, (Vector7() << 100, 70, 60, 60, 15, 22, 25).finished(),
            (Vector7() << 5, 7, 4.8, 4.8, 0.18, 0.42, 0.5).finished()),  // gains tuned on the robot
        60.0);
    system->setController(controller);
    ct::core::Integrator<14> integrator(system, ct::core::IntegrationType::RK4);

    const double dt = 0.001, lead = 4.0 / 60.0 + 0.005;
    ct::core::StateVector<14> x = state(FilteredStreamCTC::target(0.0), Vector7::Zero());
    double maxTracking = 0.0, maxToTarget = 0.0;
    for (int k = 0; k < 4000; ++k)
    {
        controller->step(k * dt, lead);
        integrator.integrate_n_steps(x, k * dt, 1, dt);
        if (k * dt >= 1.0)
        {
            maxTracking =
                std::max(maxTracking, (x.head<7>() - controller->filter().getPosition()).cwiseAbs().maxCoeff());
            maxToTarget = std::max(maxToTarget,
                (controller->filter().getPosition() - FilteredStreamCTC::target(k * dt + dt)).cwiseAbs().maxCoeff());
        }
    }
    EXPECT_LT(maxTracking, 1e-3);  // exact model: the PD only corrects the reference changing within a step
    EXPECT_LT(maxToTarget, 2e-3);  // look-ahead: the filtered reference stays on the intended path
}

TEST(ReferenceFilterTest, StepResponseWithoutOvershoot)
{
    const double wn = 60.0, dt = 0.001;
    ReferenceFilter<7> filter(wn);
    filter.reset(Vector7::Zero());
    double maxPosition = 0.0, t63 = -1.0;
    for (int k = 1; k <= 1000; ++k)
    {
        filter.update(Vector7::Ones(), dt);
        maxPosition = std::max(maxPosition, filter.getPosition()(0));
        if (t63 < 0.0 && filter.getPosition()(0) >= 1.0 - std::exp(-1.0))
            t63 = k * dt;
    }
    EXPECT_LE(maxPosition, 1.0 + 1e-6);               // critically damped: no overshoot
    EXPECT_NEAR(filter.getPosition()(0), 1.0, 1e-6);  // settled after 1 s
    EXPECT_NEAR(t63, 4.36 / wn, 0.003);               // 1 - e^-x (1 + x + x^2/2 + x^3/6) = 63 % at x = wn t = 4.36
}

TEST(ReferenceFilterTest, RampLagsByFourOverWn)
{
    const double wn = 50.0, dt = 0.001, v = 0.5;
    ReferenceFilter<7> filter(wn);
    filter.reset(Vector7::Zero());
    for (int k = 1; k <= 2000; ++k)
        filter.update(Vector7::Constant(v * k * dt), dt);
    EXPECT_NEAR(filter.getVelocity()(0), v, 1e-6);
    EXPECT_NEAR(v * 2.0 - filter.getPosition()(0), v * 4.0 / wn, 1e-3);
    EXPECT_NEAR(filter.getAcceleration()(0), 0.0, 1e-6);

    ReferenceFilter<7> copy(filter);
    EXPECT_EQ(copy.getPosition(), filter.getPosition());
    EXPECT_THROW(ReferenceFilter<7>(0.0), std::invalid_argument);
    Vector7 nan = Vector7::Zero();
    nan(1) = std::nan("");
    EXPECT_THROW(filter.update(nan, dt), std::invalid_argument);
    EXPECT_THROW(filter.update(Vector7::Zero(), 0.0), std::invalid_argument);
}
