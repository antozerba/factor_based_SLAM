#pragma once

#include "fslam/sensors/imotion_model.hpp"

namespace fslam {

// The dataset already provides the body-frame increment: u = [dx, dy, dyaw].
// Used for the GA2 simulated dataset. The covariance is a constant Q per sample
// (GA2 sim: relative_pose_cov = odom_cov_matrix, not scaled by dt).
class DirectRelativePoseModel : public IMotionModel {
public:
    explicit DirectRelativePoseModel(const Eigen::Matrix3d& Q) : Q_(Q) {}

    std::string name() const override { return "direct_relative_pose"; }
    int inputDim() const override { return 3; }
    gtsam::Pose2 relativePose(const MotionSample& s) const override;
    Eigen::Matrix3d covariance(const MotionSample&) const override { return Q_; }

private:
    Eigen::Matrix3d Q_;
};

}  // namespace fslam
