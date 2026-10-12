/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>

#include <gtest/gtest.h>

#include <ct/core/core.h>
#include "ct/models/OpenArm/OpenArm.h"

using namespace ct::models::OpenArm;

namespace {

struct ReferenceCase
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, 14, 1> q, dq, ddq, G, tau;
    Eigen::Matrix<double, 14, 14> M, C;
};

//! reference values from Pinocchio in double precision (generate_reference_values.py), urdfJointNames() order
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
            for (int k = 0; k < 14; ++k)
            {
                c.M(i, k) = v[42 + i * 14 + k];
                c.C(i, k) = v[42 + 196 + i * 14 + k];
            }
        }
        cases.push_back(c);
    }
    return cases;
}

}  // namespace

TEST(OpenArmModelTest, GeneratedCodeMatchesPinocchio)
{
    const auto cases = referenceCases();
    ASSERT_EQ(cases.size(), 40u);
    OpenArmModel<14> model;
    for (const auto& c : cases)
    {
        const ModelTerms<14> terms = model.computeTerms(c.q, c.dq);
        EXPECT_LT((terms.M - c.M).cwiseAbs().maxCoeff(), 1e-9);
        EXPECT_LT((terms.C - c.C).cwiseAbs().maxCoeff(), 1e-9);
        EXPECT_LT((terms.G - c.G).cwiseAbs().maxCoeff(), 1e-9);
        // inverse dynamics (RNEA) from the same terms
        EXPECT_LT((terms.M * c.ddq + terms.C * c.dq + terms.G - c.tau).cwiseAbs().maxCoeff(), 1e-9);
    }
}

TEST(OpenArmModelTest, OneArmInAnyJointOrder)
{
    const auto c = referenceCases().front();
    std::vector<std::string> names = armJointNames("right");
    std::reverse(names.begin(), names.end());  // J7..J1
    OpenArmModel<7> model(names);
    Eigen::Matrix<double, 7, 1> q, dq;
    for (int k = 0; k < 7; ++k)
    {
        q(k) = c.q(13 - k);
        dq(k) = c.dq(13 - k);
    }
    const ModelTerms<7> terms = model.computeTerms(q, dq);
    for (int r = 0; r < 7; ++r)
    {
        EXPECT_NEAR(terms.G(r), c.G(13 - r), 1e-9);
        for (int k = 0; k < 7; ++k)
            EXPECT_NEAR(terms.M(r, k), c.M(13 - r, 13 - k), 1e-9);
    }
    EXPECT_NEAR(model.jointLowerLimit()(6), -1.396263, 1e-9);  // right J1
    EXPECT_NEAR(model.jointUpperLimit()(0), 1.570796, 1e-9);   // right J7

    EXPECT_THROW(OpenArmModel<7>(std::vector<std::string>{"openarm_right_joint1"}), std::invalid_argument);
    names[0] = "openarm_right_joint8";
    EXPECT_THROW(OpenArmModel<7> invalid(names), std::invalid_argument);
}

TEST(OpenArmModelTest, CloneIsIndependent)
{
    OpenArmModel<14> model;
    std::unique_ptr<OpenArmModel<14>> copy(model.clone());
    const auto c = referenceCases()[5];
    EXPECT_LT((model.computeTerms(c.q, c.dq).M - copy->computeTerms(c.q, c.dq).M).cwiseAbs().maxCoeff(), 1e-15);
    EXPECT_EQ(copy->jointNames(), urdfJointNames());
}

TEST(OpenArmModelTest, SystemInControlToolboxIntegrator)
{
    // the model as a ct::core::ControlledSystem; u = G(q0) holds the arm, u = 0 lets it fall
    auto system = std::make_shared<OpenArmSystem<7>>();
    Eigen::Matrix<double, 7, 1> q0;
    q0 << 0.4, 0.1, 0.0, 0.9, 0.0, -0.2, 0.0;
    ct::core::ControlVector<7> gravity(system->model().computeTerms(q0, Eigen::Matrix<double, 7, 1>::Zero()).G);
    system->setController(std::make_shared<ct::core::ConstantController<14, 7>>(gravity));
    ct::core::Integrator<14> integrator(system, ct::core::IntegrationType::RK4);

    ct::core::StateVector<14> x;
    x << q0, Eigen::Matrix<double, 7, 1>::Zero();
    integrator.integrate_n_steps(x, 0.0, 500, 0.001);
    EXPECT_LT((x.head<7>() - q0).cwiseAbs().maxCoeff(), 1e-6);

    system->setController(std::make_shared<ct::core::ConstantController<14, 7>>());
    x << q0, Eigen::Matrix<double, 7, 1>::Zero();
    integrator.integrate_n_steps(x, 0.0, 200, 0.001);
    EXPECT_GT((x.head<7>() - q0).cwiseAbs().maxCoeff(), 1e-3);

    std::shared_ptr<OpenArmSystem<7>> copy(system->clone());
    EXPECT_EQ(copy->model().jointNames(), armJointNames("right"));
}

TEST(OpenArmModelTest, Timing)
{
    OpenArmModel<14> model;
    const auto c = referenceCases()[0];
    ModelTerms<14> terms;
    Eigen::Matrix<double, 14, 1> q = c.q;
    const int n = 20000;
    double sink = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < n; ++k)
    {
        q(0) = c.q(0) + 1e-9 * k;
        model.computeTerms(q, c.dq, terms);
        sink += terms.G(0);
    }
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / n;
    std::cout << "[ timing ] M, C, G of both arms: " << us << " us (" << sink << ")" << std::endl;
    EXPECT_LT(us, 100.0);
}
