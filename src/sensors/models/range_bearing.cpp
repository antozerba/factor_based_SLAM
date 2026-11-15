#include "fslam/sensors/models/range_bearing.hpp"

#include <stdexcept>

#include "fslam/core/math.hpp"

namespace fslam {

Eigen::Vector2d predictRangeBearing(const gtsam::Pose2& T_wb, const gtsam::Pose2& T_bs,
                                    const gtsam::Point2& l,
                                    Eigen::Matrix<double, 2, 3>* H_pose,
                                    Eigen::Matrix<double, 2, 2>* H_lm) {
    // Sensor pose in the world: T_ws = T_wb * T_bs. The mount is constant, so only
    // d T_ws / d T_wb is needed for the chain rule.
    gtsam::Matrix3 H_ws_wb;
    const gtsam::Pose2 T_ws = T_wb.compose(T_bs, H_ws_wb);

    Eigen::Matrix<double, 1, 3> Hr_ws, Hb_ws;
    Eigen::Matrix<double, 1, 2> Hr_l, Hb_l;
    const double range = T_ws.range(l, Hr_ws, Hr_l);
    const double bearing = T_ws.bearing(l, Hb_ws, Hb_l).theta();

    if (H_pose) {
        Eigen::Matrix<double, 2, 3> H_ws;
        H_ws << Hr_ws, Hb_ws;
        *H_pose = H_ws * H_ws_wb;
    }
    if (H_lm) {
        *H_lm << Hr_l, Hb_l;
    }
    return {range, bearing};
}

// ---------------------------------------------------------------------------
// RangeBearingModel
// ---------------------------------------------------------------------------

Eigen::VectorXd RangeBearingModel::predict(const gtsam::Pose2& T_wb, const gtsam::Point2& l,
                                           Eigen::MatrixXd* H_pose, Eigen::MatrixXd* H_lm) const {
    Eigen::Matrix<double, 2, 3> Hp;
    Eigen::Matrix<double, 2, 2> Hl;
    const Eigen::Vector2d z = predictRangeBearing(T_wb, mount_.T_body_sensor, l,
                                                  H_pose ? &Hp : nullptr, H_lm ? &Hl : nullptr);
    if (H_pose) *H_pose = Hp;
    if (H_lm) *H_lm = Hl;
    return z;
}

Eigen::VectorXd RangeBearingModel::residual(const Eigen::VectorXd& z, const Eigen::VectorXd& z_hat) const {
    Eigen::VectorXd r = z - z_hat;
    r(1) = math::wrapAngle(r(1));
    return r;
}

std::optional<gtsam::Point2> RangeBearingModel::inverse(const gtsam::Pose2& T_wb,
                                                        const Eigen::VectorXd& z) const {
    const double range = z(0);
    const double bearing = z(1);
    if (!(range > 0.0)) {
        return std::nullopt;
    }
    const gtsam::Point2 l_sensor = gtsam::Rot2::fromAngle(bearing).rotate(gtsam::Point2(range, 0.0));
    const gtsam::Pose2 T_ws = T_wb.compose(mount_.T_body_sensor);
    return T_ws.transformFrom(l_sensor);
}

bool RangeBearingModel::isVisible(const Eigen::VectorXd& z_hat, const FovGate& gate) const {
    return gate.contains(z_hat(0), z_hat(1));
}

gtsam::NonlinearFactor::shared_ptr RangeBearingModel::makeFactor(gtsam::Key x, gtsam::Key l,
                                                                 const Eigen::VectorXd& z,
                                                                 const gtsam::SharedNoiseModel& noise) const {
    if (z.size() != 2 || !noise || noise->dim() != 2) {
        throw std::invalid_argument("RangeBearingModel::makeFactor: z and noise must be 2-D [range, bearing]");
    }
    return boost::make_shared<RangeBearingFactor>(x, l, Eigen::Vector2d(z), noise, mount_.T_body_sensor);
}

// ---------------------------------------------------------------------------
// RangeBearingFactor
// ---------------------------------------------------------------------------

RangeBearingFactor::RangeBearingFactor(gtsam::Key x, gtsam::Key l, const Eigen::Vector2d& z,
                                       const gtsam::SharedNoiseModel& noise, const gtsam::Pose2& T_bs)
    : Base(noise, x, l), z_(z), T_bs_(T_bs) {}

gtsam::Vector RangeBearingFactor::evaluateError(const gtsam::Pose2& T_wb, const gtsam::Point2& l,
                                                boost::optional<gtsam::Matrix&> H_pose,
                                                boost::optional<gtsam::Matrix&> H_lm) const {
    Eigen::Matrix<double, 2, 3> Hp;
    Eigen::Matrix<double, 2, 2> Hl;
    const Eigen::Vector2d z_hat = predictRangeBearing(T_wb, T_bs_, l,
                                                      H_pose ? &Hp : nullptr, H_lm ? &Hl : nullptr);
    if (H_pose) *H_pose = Hp;
    if (H_lm) *H_lm = Hl;

    gtsam::Vector2 e = z_hat - z_;
    e(1) = math::wrapAngle(e(1));
    return e;
}

gtsam::NonlinearFactor::shared_ptr RangeBearingFactor::clone() const {
    return boost::make_shared<RangeBearingFactor>(*this);
}

}  // namespace fslam
