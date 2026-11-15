#include "fslam/sensors/motion/ackermann_bicycle.hpp"

#include <cmath>
#include <stdexcept>

namespace fslam {

AckermannBicycleModel::AckermannBicycleModel(const Params& p) : p_(p) {
    if (p_.wheelbase <= 0.0) {
        throw std::invalid_argument("AckermannBicycleModel: wheelbase must be > 0");
    }
}

gtsam::Pose2 AckermannBicycleModel::relativePose(const MotionSample& s) const {
    if (s.u.size() != inputDim()) {
        throw std::invalid_argument("AckermannBicycleModel: u must be [v_encoder, steering]");
    }
    const double v_encoder = s.u(0);
    const double tan_steer = std::tan(s.u(1));

    // The encoder sits H off the centre line, so it sees a different speed when turning.
    const double v_body = v_encoder / (1.0 - (p_.encoder_offset / p_.wheelbase) * tan_steer);
    const double omega = v_body * tan_steer / p_.wheelbase;

    // Exact integration of the constant body twist [v, 0, omega] over dt (an arc, not an Euler step).
    return gtsam::Pose2::Expmap(gtsam::Vector3(v_body, 0.0, omega) * s.dt());
}

Eigen::Matrix3d AckermannBicycleModel::covariance(const MotionSample& s) const {
    return p_.Q_per_second * s.dt();
}

}  // namespace fslam
