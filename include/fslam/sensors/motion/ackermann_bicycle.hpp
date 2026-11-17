#pragma once

#include "fslam/sensors/imotion_model.hpp"

namespace fslam {

// Kinematic bicycle model of the Victoria Park car, driven by a wheel encoder
// mounted off the centre line. Input u = [v_encoder (m/s), steering (rad)].
// Port of GA2 preprocessing.relative_pose; covariance(s) = Q_per_second * dt,
// which GA2 computed inside the loader (odom_cov_matrix * odo.dt).
class AckermannBicycleModel : public IMotionModel {
public:
    struct Params {
        double wheelbase = 2.83;        // L: axle distance [m]
        double encoder_offset = 0.76;   // H: centre line to wheel encoder [m]
        Eigen::Matrix3d Q_per_second = Eigen::Matrix3d::Identity();  // [x, y, yaw] per second
    };

    explicit AckermannBicycleModel(const Params& p);

    std::string name() const override { return "ackermann_bicycle"; }
    int inputDim() const override { return 2; }
    gtsam::Pose2 relativePose(const MotionSample& s) const override;
    Eigen::Matrix3d covariance(const MotionSample& s) const override;

    /// @brief Vehicle geometry and process noise.
    const Params& params() const { return p_; }

private:
    Params p_;
};

}  // namespace fslam
