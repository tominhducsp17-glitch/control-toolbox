/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cppad/cg.hpp>

#include <string>
#include <vector>

/*!
 * Pinocchio side of the OpenArm code generator (OpenArmModelCodegen.cpp).
 *
 * Pinocchio and ct_core both define Eigen traits for the CppAD scalar, so the two sides are compiled
 * separately and exchange plain std::vector values only.
 */
namespace ct {
namespace models {
namespace OpenArm {
namespace codegen {

static const size_t NJOINTS_ARM = 7;
static const size_t IN_DIM = 2 * NJOINTS_ARM;                               //!< x = [q; dq]
static const size_t OUT_DIM = 2 * NJOINTS_ARM * NJOINTS_ARM + NJOINTS_ARM;  //!< y = [M; C; G], row-major

typedef CppAD::AD<CppAD::cg::CG<double>> ADCGScalar;

//! load the official OpenArm URDF (once)
void loadUrdf(const std::string& urdfPath);

//! M(q), C(q, dq) and G(q) of one arm ("right" or "left"), other joints fixed at 0
std::vector<ADCGScalar> modelTerms(const std::string& arm, const std::vector<ADCGScalar>& x);

//! URDF position limits of the 7 joints of one arm
std::vector<double> positionLimits(const std::string& arm, bool upper);

}  // namespace codegen
}  // namespace OpenArm
}  // namespace models
}  // namespace ct
