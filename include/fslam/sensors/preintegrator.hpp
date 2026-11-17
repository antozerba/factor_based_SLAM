#pragma once

#include <cstddef>

#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>

#include "fslam/core/types.hpp"
#include "fslam/sensors/imotion_model.hpp"

namespace fslam {

// Compounds many small pose increments into one RelativeMotion and propagates
// its covariance:  delta <- delta * inc,  cov <- H1 cov H1^T + H2 Q H2^T.
// Port of GA2 preprocessing.preintegrate, but incremental: call integrate() for
// every sample as it arrives, result() at a keyframe, then reset().
class Preintegrator {
public:
    // Start a new interval at time t (identity increment, zero covariance).
    void reset(Timestamp t);

    // Add one increment with covariance inc_cov that ends at time t1.
    void integrate(const gtsam::Pose2& inc, const Eigen::Matrix3d& inc_cov, Timestamp t1);

    // Convenience: run the motion model on the sample, then integrate.
    void integrate(const IMotionModel& model, const MotionSample& s);

    /// @brief True if no increment has been integrated since the last reset().
    bool empty() const { return count_ == 0; }
    /// @brief Number of increments integrated since the last reset().
    std::size_t count() const { return count_; }

    /// @brief Accumulated increment ΔT, body frame at t0.
    const gtsam::Pose2& delta() const { return delta_; }
    /// @brief Covariance P of delta(), tangent space, ordered [x, y, yaw].
    const Eigen::Matrix3d& covariance() const { return cov_; }

    RelativeMotion result() const;

private:
    gtsam::Pose2 delta_;
    Eigen::Matrix3d cov_ = Eigen::Matrix3d::Zero();
    Timestamp t0_ = 0.0;
    Timestamp t1_ = 0.0;
    std::size_t count_ = 0;
    bool started_ = false;
};

}  // namespace fslam
