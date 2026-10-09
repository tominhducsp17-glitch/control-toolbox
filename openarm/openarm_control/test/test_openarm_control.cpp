// Generated model against Pinocchio reference values, the virtual controller interface, and the
// control-toolbox adapters.
#include "openarm_control/controller.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>

using namespace openarm_control;

namespace
{
std::vector<std::string> bimanual()
{
  std::vector<std::string> n;
  for (const char * side : {"left", "right"}) {
    for (int j = 1; j <= 7; ++j) {n.push_back(std::string("openarm_") + side + "_joint" + std::to_string(j));}
  }
  return n;
}

struct GoldenCase
{
  JointVector<14> q, dq, ddq, G, tau;
  JointMatrix<14> M, C;
};

// Rows: q dq ddq (14 each), M (196 row-major), C (196 row-major), G (14), tau_id (14). Pinocchio, double.
std::vector<GoldenCase> golden()
{
  std::ifstream file(GOLDEN_CSV);
  std::vector<GoldenCase> cases;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::stringstream in(line);
    std::vector<double> v;
    double x;
    while (in >> x) {v.push_back(x);}
    GoldenCase c;
    for (int i = 0; i < 14; ++i) {
      c.q[i] = v[i];
      c.dq[i] = v[14 + i];
      c.ddq[i] = v[28 + i];
      c.G[i] = v[42 + 392 + i];
      c.tau[i] = v[42 + 392 + 14 + i];
      for (int k = 0; k < 14; ++k) {
        c.M(i, k) = v[42 + i * 14 + k];
        c.C(i, k) = v[42 + 196 + i * 14 + k];
      }
    }
    cases.push_back(c);
  }
  return cases;
}
}  // namespace

TEST(OpenArmModel, GeneratedCodeMatchesPinocchio)
{
  const auto cases = golden();
  ASSERT_EQ(cases.size(), 40u);
  const OpenArmModel<14> model(bimanual());
  for (const auto & c : cases) {
    const auto m = model.compute(c.q, c.dq);
    EXPECT_LT((m.M - c.M).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((m.C - c.C).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((m.G - c.G).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((m.M * c.ddq + m.C * c.dq + m.G - c.tau).cwiseAbs().maxCoeff(), 1e-9);  // = RNEA
  }
}

TEST(OpenArmModel, OneArmAnyOrderAndLimits)
{
  const auto c = golden().front();
  // Right arm only, joints listed in reverse order.
  std::vector<std::string> names;
  for (int j = 7; j >= 1; --j) {names.push_back("openarm_right_joint" + std::to_string(j));}
  const OpenArmModel<7> model(names);
  JointVector<7> q, dq;
  for (int k = 0; k < 7; ++k) {
    q[k] = c.q[7 + 6 - k];
    dq[k] = c.dq[7 + 6 - k];
  }
  const auto m = model.compute(q, dq);
  for (int r = 0; r < 7; ++r) {
    EXPECT_NEAR(m.G[r], c.G[7 + 6 - r], 1e-9);
    for (int k = 0; k < 7; ++k) {EXPECT_NEAR(m.M(r, k), c.M(7 + 6 - r, 7 + 6 - k), 1e-9);}
  }
  EXPECT_NEAR(model.upperLimits()[0], 1.570796, 1e-9);   // joint7
  EXPECT_NEAR(model.lowerLimits()[6], -1.396263, 1e-9);  // right joint1
  EXPECT_THROW(OpenArmModel<7>({"openarm_right_joint1"}), std::invalid_argument);
  auto bad = names;
  bad[0] = "openarm_right_joint8";
  EXPECT_THROW(OpenArmModel<7> m2(bad), std::invalid_argument);
}

TEST(Controller, CtcFeedforwardIsInverseDynamicsAtReference)
{
  const OpenArmModel<14> model(bimanual());
  auto ctc = makeController<14>("ctc_feedforward");
  auto grav = makeController<14>("gravity_compensation");
  for (const auto & c : golden()) {
    ControllerInput<14> in;
    in.q_d = c.q;
    in.dq_d = c.dq;
    in.ddq_d = c.ddq;
    in.q = c.q + JointVector<14>::Constant(0.05);  // measured state differs: must not matter
    computeModelTerms(model, in);
    EXPECT_LT((ctc->compute(in) - c.tau).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((grav->compute(in) - c.G).cwiseAbs().maxCoeff(), 1e-9);
  }
}

// A controller written outside the library: override computeTorque() and use the common input.
template<size_t NJ>
class PdPlusGravity : public ControllerBase<NJ>
{
public:
  JointVector<NJ> computeTorque(const ControllerInput<NJ> & in) override
  {
    return 50.0 * (in.q_d - in.q) + 2.0 * (in.dq_d - in.dq) + in.measured.G;
  }
  std::string name() const override {return "pd_plus_gravity";}
  std::unique_ptr<ControllerBase<NJ>> clone() const override {return std::make_unique<PdPlusGravity>(*this);}
};

TEST(Controller, NewControllerByOverride)
{
  const OpenArmModel<14> model(bimanual());
  std::unique_ptr<ControllerBase<14>> controller = std::make_unique<PdPlusGravity<14>>();
  const auto c = golden()[3];
  ControllerInput<14> in;
  in.q = c.q;
  in.dq = c.dq;
  in.q_d = c.q + JointVector<14>::Constant(0.01);
  computeModelTerms(model, in);
  const JointVector<14> expected = 50.0 * JointVector<14>::Constant(0.01) - 2.0 * c.dq + c.G;
  EXPECT_LT((controller->compute(in) - expected).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_EQ(controller->clone()->name(), "pd_plus_gravity");
}

TEST(Controller, TorqueLimitAndRejection)
{
  const OpenArmModel<14> model(bimanual());
  auto ctc = makeController<14>("ctc_feedforward");
  ctc->setTorqueLimit(std::vector<double>(14, 0.5));
  ControllerInput<14> in;
  in.q_d = golden()[0].q;
  computeModelTerms(model, in);
  EXPECT_LE(ctc->compute(in).cwiseAbs().maxCoeff(), 0.5);
  EXPECT_TRUE(ctc->lastClamped());
  in.ddq_d[2] = std::nan("");
  EXPECT_THROW(ctc->compute(in), std::invalid_argument);
  EXPECT_THROW(ctc->setTorqueLimit({1.0}), std::invalid_argument);
  EXPECT_THROW(makeController<14>("mpc"), std::invalid_argument);
}

TEST(ControlToolbox, SimulationWithIntegrator)
{
  // ct::core: OpenArmDynamics (generated model) + a ControllerBase through CtControllerAdapter.
  std::vector<std::string> right;
  for (int j = 1; j <= 7; ++j) {right.push_back("openarm_right_joint" + std::to_string(j));}
  auto system = std::make_shared<OpenArmDynamics<7>>(right);
  auto hold = std::make_shared<CtControllerAdapter<7>>(
    std::shared_ptr<ControllerBase<7>>(makeController<7>("gravity_compensation")), system->model());
  JointVector<7> q0;
  q0 << 0.4, 0.1, 0.0, 0.9, 0.0, -0.2, 0.0;
  hold->setReference(q0, JointVector<7>::Zero(), JointVector<7>::Zero());
  system->setController(hold);
  ct::core::Integrator<14> integrator(system, ct::core::IntegrationType::RK4);
  State<7> x;
  x << q0, JointVector<7>::Zero();
  integrator.integrate_n_steps(x, 0.0, 500, 0.001);
  EXPECT_LT((x.head<7>() - q0).cwiseAbs().maxCoeff(), 1e-6);  // u = G(q0) holds the arm

  system->setController(std::make_shared<ct::core::ConstantController<14, 7>>());
  State<7> fall;
  fall << q0, JointVector<7>::Zero();
  integrator.integrate_n_steps(fall, 0.0, 200, 0.001);
  EXPECT_GT((fall.head<7>() - q0).cwiseAbs().maxCoeff(), 1e-3);  // u = 0: it falls
}

TEST(Controller, AccelerationEstimator)
{
  AccelerationEstimator<7> est(5.0, 20.0);
  JointVector<7> dq = JointVector<7>::Zero();
  EXPECT_EQ(est.update(0.0, dq).norm(), 0.0);
  for (int k = 1; k <= 400; ++k) {
    dq.setConstant(0.5 * k / 400.0);
    est.update(k / 400.0, dq);
  }
  EXPECT_NEAR(est.update(1.0, dq)[0], 0.5, 0.01);
}

TEST(OpenArmModel, Timing)
{
  const OpenArmModel<14> model(bimanual());
  const auto c = golden()[0];
  ControllerInput<14> in;
  in.q = in.q_d = c.q;
  in.dq = in.dq_d = c.dq;
  auto ctc = makeController<14>("ctc_feedforward");
  const int n = 20000;
  double sink = 0.0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int k = 0; k < n; ++k) {
    in.q_d[0] = c.q[0] + 1e-9 * k;
    computeModelTerms(model, in);  // M, C, G of both arms at two states
    sink += ctc->compute(in)[0];
  }
  const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / n;
  std::cout << "[ timing ] model terms (2 states x 2 arms) + CTC: " << us << " us per cycle (" << sink << ")\n";
  EXPECT_LT(us, 200.0);  // far inside the 2500 us cycle at 400 Hz
}
