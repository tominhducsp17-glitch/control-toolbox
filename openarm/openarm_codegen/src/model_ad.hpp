// Pinocchio side and control-toolbox side are compiled in separate files: both define Eigen traits
// for the CppAD scalar, so they exchange plain std::vector values only.
#pragma once

#include <cppad/cg.hpp>

#include <string>
#include <vector>

namespace openarm_codegen
{
constexpr int kJoints = 7;
constexpr int kIn = 2 * kJoints;                          // x = [q; dq]
constexpr int kOut = 2 * kJoints * kJoints + kJoints;     // y = [M row-major; C row-major; G]
using ADCG = CppAD::AD<CppAD::cg::CG<double>>;

// Loads the URDF once; arm = "right" | "left".
void loadModel(const std::string & urdf_path);
std::vector<ADCG> modelTerms(const std::string & arm, const std::vector<ADCG> & x);
// URDF position limits of the 7 joints of one arm.
std::vector<double> positionLimits(const std::string & arm, bool upper);
}  // namespace openarm_codegen
