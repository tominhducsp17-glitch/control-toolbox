/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/math/Derivatives.h>

namespace ct {
namespace openarm {
namespace generated {

class OpenArmRightModel : public core::Derivatives<14, 105, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, 105, 1> OUT_TYPE;
    typedef Eigen::Matrix<double, 14, 1> X_TYPE;

    OpenArmRightModel()
    {
        eval_.setZero();
        v_.fill(0.0);
    };

    OpenArmRightModel(const OpenArmRightModel& other)
    {
        eval_.setZero();
        v_.fill(0.0);
    }

    virtual ~OpenArmRightModel(){};

    OpenArmRightModel* clone() const override { return new OpenArmRightModel(*this); }
    OUT_TYPE forwardZero(const Eigen::VectorXd& x_in) override;

private:
    OUT_TYPE eval_;
    std::array<double, 619> v_;
};

}  // namespace generated
}  // namespace openarm
}  // namespace ct
