#include "openarm_control/openarm_model.hpp"

#include "OpenArmLeftModel.h"
#include "OpenArmLimits.h"
#include "OpenArmRightModel.h"

#include <array>
#include <set>
#include <stdexcept>

namespace openarm_control
{

namespace gen = ct::openarm::generated;

template<size_t NJ>
struct OpenArmModel<NJ>::Impl
{
  std::vector<std::string> names;
  // For each arm present: index of its joint 1..7 in the NJ-vector.
  bool has_right = false, has_left = false;
  std::array<int, 7> right{}, left{};
  mutable gen::OpenArmRightModel right_model;
  mutable gen::OpenArmLeftModel left_model;
  mutable Eigen::VectorXd x = Eigen::VectorXd::Zero(14);  // generated code input [q; dq], no allocation per call

  explicit Impl(const std::vector<std::string> & joint_names)
  : names(joint_names)
  {
    if (names.size() != NJ) {
      throw std::invalid_argument("OpenArmModel: expected " + std::to_string(NJ) + " joint names");
    }
    if (std::set<std::string>(names.begin(), names.end()).size() != NJ) {
      throw std::invalid_argument("OpenArmModel: duplicate joint names");
    }
    std::array<int, 7> r, l;
    r.fill(-1);
    l.fill(-1);
    for (size_t k = 0; k < NJ; ++k) {
      const std::string & n = names[k];
      for (const auto & [prefix, slots] : {std::pair<std::string, std::array<int, 7> *>{"openarm_right_joint", &r},
          {"openarm_left_joint", &l}})
      {
        if (n.rfind(prefix, 0) == 0 && n.size() == prefix.size() + 1 && n.back() >= '1' && n.back() <= '7') {
          (*slots)[n.back() - '1'] = static_cast<int>(k);
        }
      }
    }
    const auto complete = [](const std::array<int, 7> & a) {
        int n = 0;
        for (int v : a) {n += v >= 0;}
        return n;
      };
    if (complete(r) + complete(l) != static_cast<int>(NJ) || (complete(r) % 7) || (complete(l) % 7)) {
      throw std::invalid_argument(
              "OpenArmModel: joint names must be openarm_<right|left>_joint1..7, each arm complete");
    }
    has_right = complete(r) == 7;
    has_left = complete(l) == 7;
    right = r;
    left = l;
  }

  template<typename Generated>
  void addArm(
    Generated & model, const std::array<int, 7> & idx, const JointVector<NJ> & q,
    const JointVector<NJ> & dq, ModelTerms<NJ> & out) const
  {
    for (int j = 0; j < 7; ++j) {
      x[j] = q[idx[j]];
      x[7 + j] = dq[idx[j]];
    }
    const auto y = model.forwardZero(x);  // [M row-major (49); C row-major (49); G (7)]
    for (int r = 0; r < 7; ++r) {
      for (int c = 0; c < 7; ++c) {
        out.M(idx[r], idx[c]) = y[r * 7 + c];
        out.C(idx[r], idx[c]) = y[49 + r * 7 + c];
      }
      out.G[idx[r]] = y[98 + r];
    }
  }
};

template<size_t NJ>
OpenArmModel<NJ>::OpenArmModel(const std::vector<std::string> & joint_names)
: impl_(std::make_unique<Impl>(joint_names)) {}

template<size_t NJ>
OpenArmModel<NJ>::OpenArmModel(const OpenArmModel & other)
: impl_(std::make_unique<Impl>(other.impl_->names)) {}

template<size_t NJ>
OpenArmModel<NJ>::~OpenArmModel() = default;

template<size_t NJ>
ModelTerms<NJ> OpenArmModel<NJ>::compute(const JointVector<NJ> & q, const JointVector<NJ> & dq) const
{
  if (!q.allFinite() || !dq.allFinite()) {throw std::invalid_argument("OpenArmModel: non-finite input");}
  ModelTerms<NJ> out;  // zero blocks between the arms
  if (impl_->has_right) {impl_->addArm(impl_->right_model, impl_->right, q, dq, out);}
  if (impl_->has_left) {impl_->addArm(impl_->left_model, impl_->left, q, dq, out);}
  return out;
}

template<size_t NJ>
const std::vector<std::string> & OpenArmModel<NJ>::jointNames() const {return impl_->names;}

template<size_t NJ>
JointVector<NJ> OpenArmModel<NJ>::lowerLimits() const
{
  JointVector<NJ> out;
  for (int j = 0; j < 7; ++j) {
    if (impl_->has_right) {out[impl_->right[j]] = gen::kRightLower[j];}
    if (impl_->has_left) {out[impl_->left[j]] = gen::kLeftLower[j];}
  }
  return out;
}

template<size_t NJ>
JointVector<NJ> OpenArmModel<NJ>::upperLimits() const
{
  JointVector<NJ> out;
  for (int j = 0; j < 7; ++j) {
    if (impl_->has_right) {out[impl_->right[j]] = gen::kRightUpper[j];}
    if (impl_->has_left) {out[impl_->left[j]] = gen::kLeftUpper[j];}
  }
  return out;
}

template<size_t NJ>
OpenArmDynamics<NJ>::OpenArmDynamics(const std::vector<std::string> & joint_names)
: model_(joint_names) {}

template<size_t NJ>
void OpenArmDynamics<NJ>::computeControlledDynamics(
  const State<NJ> & state, const double & /*t*/, const Torque<NJ> & control,
  ct::core::StateVector<2 * NJ> & derivative)
{
  const JointVector<NJ> q = state.template head<NJ>();
  const JointVector<NJ> dq = state.template tail<NJ>();
  const ModelTerms<NJ> m = model_.compute(q, dq);
  derivative.template head<NJ>() = dq;
  derivative.template tail<NJ>() = m.M.ldlt().solve(JointVector<NJ>(control) - m.C * dq - m.G);
}

template class OpenArmModel<7>;
template class OpenArmModel<14>;
template class OpenArmDynamics<7>;
template class OpenArmDynamics<14>;

}  // namespace openarm_control
