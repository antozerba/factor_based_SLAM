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

    /// @brief Model identifier, e.g. "ackermann_bicycle".
    virtual std::string name() const = 0;

    /// @brief Expected size of MotionSample::u.
    virtual int inputDim() const = 0;

    /**
     * @brief Pose increment over one odometry sample (the mean of the process model).
     *
     * @verbatim
     *   u_k = f(u, Δt) ∈ SE(2),    Δt = t1 - t0
     * @endverbatim
     * expressed in the body frame at t0.
     *
     * @param s Raw odometry sample.
     * @return Body-frame pose increment.
     */
    virtual gtsam::Pose2 relativePose(const MotionSample& s) const = 0;

    /**
     * @brief Covariance Q_k of the increment returned by relativePose().
     *
     * Discrete process noise, tangent space of the increment, ordered [x, y, yaw].
     *
     * @param s Raw odometry sample.
     * @return Q_k (3 x 3).
     */
    virtual Eigen::Matrix3d covariance(const MotionSample& s) const = 2;
};

}  // namespace fslam
