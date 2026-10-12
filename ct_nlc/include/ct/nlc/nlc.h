/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

// nonlinear (model-based) controllers, all of them ct::core::Controller
#include <ct/core/core.h>

#include "control/ModelBasedJointController.h"
#include "control/ComputedTorqueController.h"
#include "control/GravityCompensationController.h"
#include "utils/ReferenceAccelerationEstimator.h"

#include "control/ModelBasedJointController-impl.h"
#include "control/ComputedTorqueController-impl.h"
#include "control/GravityCompensationController-impl.h"
#include "utils/ReferenceAccelerationEstimator-impl.h"
