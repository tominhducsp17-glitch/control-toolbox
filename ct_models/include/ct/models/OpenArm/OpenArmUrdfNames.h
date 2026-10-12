/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <string>
#include <vector>

namespace ct {
namespace models {
namespace OpenArm {

//! the 14 arm joints of the official OpenArm v1 bimanual URDF: left J1..J7, then right J1..J7
inline const std::vector<std::string>& urdfJointNames()
{
    static const std::vector<std::string> urdfJointNames{"openarm_left_joint1", "openarm_left_joint2",
        "openarm_left_joint3", "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
        "openarm_left_joint7", "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
        "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6", "openarm_right_joint7"};
    return urdfJointNames;
}

//! joints J1..J7 of one arm ("left" or "right")
inline std::vector<std::string> armJointNames(const std::string& arm)
{
    std::vector<std::string> names;
    for (int j = 1; j <= 7; ++j)
        names.push_back("openarm_" + arm + "_joint" + std::to_string(j));
    return names;
}

}  // namespace OpenArm
}  // namespace models
}  // namespace ct
