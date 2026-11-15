#pragma once

#include <string>

#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>

#include "fslam/core/types.hpp"

namespace fslam {

// Turns one raw odometry reading (MotionSample) into a body-frame pose increment
// and its covariance. The meaning of sample.u is defined by each implementation.
class IMotionModel {
public:
    virtual ~IMotionModel() = default;

    virtual std::string name() const = 0;

    // Expected size of MotionSample::u.
    virtual int inputDim() const = 0;

    // Pose increment over [s.t0, s.t1], expressed in the body frame at s.t0.
    virtual gtsam::Pose2 relativePose(const MotionSample& s) const = 0;

    // Covariance of that increment, in its tangent space, ordered [x, y, yaw].
    virtual Eigen::Matrix3d covariance(const MotionSample& s) const = 0;
};

}  // namespace fslam
