/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

/*!
 * OpenArm v1 bimanual robot (official URDF: ct_models/urdf/OpenArm/openarm_v1_bimanual.urdf).
 *
 * The joint-space model (M, C, G of each arm) is generated code in codegen/, written by OpenArmModelCodegen
 * (src/OpenArm/codegen) with Pinocchio and ct::core::DerivativesCppadCG. Link the library OpenArmModel.
 */

#include <ct/models/OpenArm/OpenArmUrdfNames.h>
#include <ct/models/OpenArm/OpenArmModel.h>
#include <ct/models/OpenArm/OpenArmModel-impl.h>
