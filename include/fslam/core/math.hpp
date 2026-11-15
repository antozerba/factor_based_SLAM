#pragma once
#include <Eigen/Core>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>

namespace fslam::math
{

    double wrapAngle(double angle);

    Eigen::MatrixXd assembleJointCovariance(const gtsam::JointMarginal& joint_marginal,
        const gtsam::KeyVector& keys) ;

} // namespace fslam::math