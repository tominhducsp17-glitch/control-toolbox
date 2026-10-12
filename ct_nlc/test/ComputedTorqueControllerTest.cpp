/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

#include <gtest/gtest.h>

#include <ct/nlc/nlc.h>
#include <ct/models/OpenArm/OpenArm.h>

using namespace ct::nlc;
using ct::models::OpenArm::OpenArmModel;
using ct::models::OpenArm::OpenArmSystem;

typedef Eigen::Matrix<double, 7, 1> Vector7;
typedef Eigen::Matrix<double, 14, 1> Vector14;

namespace {

struct ReferenceCase
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector14 q, dq, ddq, G, tau;
};

//! reference values from Pinocchio (ct_models/test/OpenArm/data), urdfJointNames() order
std::vector<ReferenceCase, Eigen::aligned_allocator<ReferenceCase>> referenceCases()
{
    std::ifstream file(OPENARM_REFERENCE_CSV);
    std::vector<ReferenceCase, Eigen::aligned_allocator<ReferenceCase>> cases;
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::stringstream in(line);
        std::vector<double> v;
        double x;
        while (in >> x)
            v.push_back(x);
        ReferenceCase c;
        for (int i = 0; i < 14; ++i)
        {
            c.q(i) = v[i];
            c.dq(i) = v[14 + i];
            c.ddq(i) = v[28 + i];
            c.G(i) = v[42 + 392 + i];
            c.tau(i) = v[42 + 392 + 14 + i];
        }
        cases.push_back(c);
    }
    return cases;
}

ct::core::StateVector<14> state(const Vector7& q, const Vector7& dq)
{
    ct::core::StateVector<14> x;
    x << q, dq;
    return x;
}

}  // namespace

TEST(ComputedTorqueControllerTest, FeedforwardIsInverseDynamicsOfTheReference)
{
    const auto cases = referenceCases();
    ASSERT_EQ(cases.size(), 40u);
    ComputedTorqueController<14, OpenArmModel<14>> ctc(OpenArmModel<14>{});
    GravityCompensationController<14, OpenArmModel<14>> gravity(OpenArmModel<14>{});
    ct::core::ControlVector<14> u;
    for (const auto& c : cases)
    {
        ctc.setReference(c.q, c.dq, c.ddq);
        gravity.setReference(c.q, c.dq, c.ddq);
        // the measured state differs from the reference: the feedforward must not depend on it
        ct::core::StateVector<28> x;
        x << c.q + Vector14::Constant(0.05), c.dq - Vector14::Constant(0.1);
        ctc.computeControl(x, 0.0, u);
        EXPECT_LT((u - c.tau).cwiseAbs().maxCoeff(), 1e-9);
        gravity.computeControl(x, 0.0, u);
        EXPECT_LT((u - c.G).cwiseAbs().maxCoeff(), 1e-9);
    }
}

TEST(ComputedTorqueControllerTest, FeedbackOnTheMeasuredState)
{
    const auto c = referenceCases()[7];
    const Vector7 q = c.q.tail<7>(), dq = c.dq.tail<7>();
    const Vector7 Kp = Vector7::Constant(100.0), Kd = Vector7::Constant(20.0);
    OpenArmModel<7> model;
    ComputedTorqueController<7, OpenArmModel<7>> ctc(model, Kp, Kd, ModelEvaluation::MEASURED);
    const Vector7 q_d = q + Vector7::Constant(0.02), dq_d = dq - Vector7::Constant(0.1);
    const Vector7 ddq_d = c.ddq.tail<7>();
    ctc.setReference(q_d, dq_d, ddq_d);
    ct::core::ControlVector<7> u;
    ctc.computeControl(state(q, dq), 0.0, u);

    const auto terms = model.computeTerms(q, dq);
    const Vector7 ddq_cmd = ddq_d + Kp.cwiseProduct(q_d - q) + Kd.cwiseProduct(dq_d - dq);
    EXPECT_LT((u - (terms.M * ddq_cmd + terms.C * dq + terms.G)).cwiseAbs().maxCoeff(), 1e-9);
    // with the reference equal to the state, u is the inverse dynamics at the state
    ctc.setReference(q, dq, c.ddq.tail<7>());
    ctc.computeControl(state(q, dq), 0.0, u);
    EXPECT_LT((u - c.tau.tail<7>()).cwiseAbs().maxCoeff(), 1e-9);
}

//! a controller written outside ct_nlc: PD plus gravity at the measured state, by overriding computeControl()
template <size_t NJOINTS, class MODEL>
class PDGravityController : public ModelBasedJointController<NJOINTS, MODEL>
{
public:
    typedef ModelBasedJointController<NJOINTS, MODEL> Base;

    PDGravityController(const MODEL& model, double kp, double kd) : Base(model), kp_(kp), kd_(kd) {}
    PDGravityController(const PDGravityController& arg) : Base(arg), kp_(arg.kp_), kd_(arg.kd_) {}
    virtual ~PDGravityController() {}
    virtual PDGravityController* clone() const override { return new PDGravityController(*this); }

    virtual void computeControl(const typename Base::state_vector_t& state,
        const ct::core::Time& t,
        typename Base::control_vector_t& control) override
    {
        const typename Base::joint_vector_t q = state.template head<NJOINTS>();
        this->model_->computeTerms(q, state.template tail<NJOINTS>(), this->terms_);
        control = kp_ * (this->q_d_ - q) + kd_ * (this->dq_d_ - state.template tail<NJOINTS>()) + this->terms_.G;
        this->saturate(control);
    }

private:
    double kp_;
    double kd_;
};

TEST(ComputedTorqueControllerTest, NewControllerByOverride)
{
    const auto c = referenceCases()[3];
    const Vector7 q = c.q.tail<7>(), dq = c.dq.tail<7>();
    std::shared_ptr<ct::core::Controller<14, 7>> controller(
        new PDGravityController<7, OpenArmModel<7>>(OpenArmModel<7>{}, 50.0, 2.0));
    std::static_pointer_cast<PDGravityController<7, OpenArmModel<7>>>(controller)
        ->setReference(q + Vector7::Constant(0.01), Vector7::Zero(), Vector7::Zero());
    ct::core::ControlVector<7> u;
    controller->computeControl(state(q, dq), 0.0, u);  // through the ct::core::Controller interface
    EXPECT_LT((u - (50.0 * Vector7::Constant(0.01) - 2.0 * dq + c.G.tail<7>())).cwiseAbs().maxCoeff(), 1e-9);

    std::shared_ptr<ct::core::Controller<14, 7>> copy(controller->clone());
    ct::core::ControlVector<7> uCopy;
    copy->computeControl(state(q, dq), 0.0, uCopy);
    EXPECT_EQ(u, uCopy);
}

TEST(ComputedTorqueControllerTest, CloneIsIndependent)
{
    const auto c = referenceCases()[5];
    ComputedTorqueController<14, OpenArmModel<14>> ctc(OpenArmModel<14>{});
    ctc.setReference(c.q, c.dq, c.ddq);
    std::unique_ptr<ComputedTorqueController<14, OpenArmModel<14>>> copy(ctc.clone());
    ctc.setReference(Vector14::Zero(), Vector14::Zero(), Vector14::Zero());
    ct::core::ControlVector<14> u;
    copy->computeControl(ct::core::StateVector<28>::Zero(), 0.0, u);
    EXPECT_LT((u - c.tau).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_NE(&copy->model(), &ctc.model());
    EXPECT_EQ(copy->getModelEvaluation(), ModelEvaluation::REFERENCE);
}

TEST(ComputedTorqueControllerTest, TorqueLimitAndInvalidInput)
{
    ComputedTorqueController<14, OpenArmModel<14>> ctc(OpenArmModel<14>{});
    ctc.setTorqueLimit(Vector14::Constant(0.5));
    const auto c = referenceCases()[0];
    ctc.setReference(c.q, c.dq, c.ddq);
    ct::core::ControlVector<14> u;
    ctc.computeControl(ct::core::StateVector<28>::Zero(), 0.0, u);
    EXPECT_LE(u.cwiseAbs().maxCoeff(), 0.5);
    EXPECT_TRUE(ctc.lastControlSaturated());

    Vector14 nan = c.ddq;
    nan(2) = std::nan("");
    EXPECT_THROW(ctc.setReference(c.q, c.dq, nan), std::invalid_argument);
    EXPECT_THROW(ctc.setTorqueLimit(Vector14::Zero()), std::invalid_argument);
    EXPECT_THROW(ctc.setGains(-Vector14::Ones(), Vector14::Ones()), std::invalid_argument);
}

//! sine reference of the right arm, evaluated at the time of each controller call (also inside the RK4 steps)
class SineReferenceCTC : public ct::core::Controller<14, 7>
{
public:
    typedef ComputedTorqueController<7, OpenArmModel<7>> CTC;

    SineReferenceCTC(const CTC& ctc) : ctc_(ctc.clone())
    {
        q0_ << 0.4, 0.1, 0.0, 0.9, 0.0, -0.2, 0.0;
        amplitude_ << 0.3, 0.2, 0.3, 0.3, 0.3, 0.2, 0.3;
    }
    SineReferenceCTC(const SineReferenceCTC& arg)
        : ct::core::Controller<14, 7>(arg), ctc_(arg.ctc_->clone()), q0_(arg.q0_), amplitude_(arg.amplitude_)
    {
    }
    virtual ~SineReferenceCTC() {}
    virtual SineReferenceCTC* clone() const override { return new SineReferenceCTC(*this); }

    virtual void computeControl(const ct::core::StateVector<14>& state,
        const ct::core::Time& t,
        ct::core::ControlVector<7>& control) override
    {
        ctc_->setReference(position(t), amplitude_ * w_ * std::cos(w_ * t), -amplitude_ * w_ * w_ * std::sin(w_ * t));
        ctc_->computeControl(state, t, control);
    }

    Vector7 position(double t) const { return q0_ + amplitude_ * std::sin(w_ * t); }

private:
    std::shared_ptr<CTC> ctc_;
    Vector7 q0_;
    Vector7 amplitude_;
    const double w_ = 2.0 * M_PI * 0.5;
};

TEST(ComputedTorqueControllerTest, TracksTrajectoryInSimulation)
{
    // OpenArm right arm as a ct::core::ControlledSystem, controlled by computed-torque control
    auto system = std::make_shared<OpenArmSystem<7>>();
    auto controller = std::make_shared<SineReferenceCTC>(SineReferenceCTC::CTC(
        OpenArmModel<7>{}, Vector7::Constant(100.0), Vector7::Constant(20.0), ModelEvaluation::MEASURED));
    system->setController(controller);
    ct::core::Integrator<14> integrator(system, ct::core::IntegrationType::RK4);

    const double dt = 0.001;
    ct::core::StateVector<14> x = state(controller->position(0.0) + Vector7::Constant(0.05), Vector7::Zero());
    double maxErrorAfter1s = 0.0;
    for (int k = 1; k <= 3000; ++k)
    {
        integrator.integrate_n_steps(x, (k - 1) * dt, 1, dt);
        if (k * dt >= 1.0)
            maxErrorAfter1s =
                std::max(maxErrorAfter1s, (x.head<7>() - controller->position(k * dt)).cwiseAbs().maxCoeff());
    }
    // exact model: the error of 0.05 rad decays like (1 + 10 t) e^(-10 t), 2.5e-5 rad after 1 s
    EXPECT_LT(maxErrorAfter1s, 3e-5);
}

TEST(ReferenceAccelerationEstimatorTest, ConvergesToTheAcceleration)
{
    ReferenceAccelerationEstimator<7> estimator(5.0, 20.0);
    Vector7 dq = Vector7::Zero();
    EXPECT_EQ(estimator.update(0.0, dq).norm(), 0.0);
    for (int k = 1; k <= 400; ++k)
    {
        dq.setConstant(0.5 * k / 400.0);
        estimator.update(k / 400.0, dq);
    }
    EXPECT_NEAR(estimator.getEstimate()(0), 0.5, 0.01);
    estimator.reset();
    EXPECT_EQ(estimator.getEstimate().norm(), 0.0);
    EXPECT_THROW(ReferenceAccelerationEstimator<7>(0.0, 1.0), std::invalid_argument);
}

TEST(ComputedTorqueControllerTest, Timing)
{
    ComputedTorqueController<14, OpenArmModel<14>> ctc(OpenArmModel<14>{});
    const auto c = referenceCases()[0];
    ct::core::StateVector<28> x;
    x << c.q, c.dq;
    ct::core::ControlVector<14> u;
    const int n = 20000;
    double sink = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < n; ++k)
    {
        ctc.setReference(c.q + Vector14::Constant(1e-9 * k), c.dq, c.ddq);
        ctc.computeControl(x, 0.0, u);
        sink += u(0);
    }
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / n;
    std::cout << "[ timing ] computed-torque control, both arms: " << us << " us (" << sink << ")" << std::endl;
    EXPECT_LT(us, 100.0);  // far inside the 2500 us cycle at 400 Hz
}
