#include "fslam/sensors/models/range_bearing.hpp"

#include <stdexcept>

#include "fslam/core/math.hpp"

namespace fslam {

/**
 * @brief Range-bearing measurement function with a sensor mount, plus Jacobians.
 *
 * Sensor pose in the world, with position p_s and heading ψ_s:
 * @verbatim
 *   T_ws = T_wb ⊕ T_bs
 *   δ    = l - p_s = [δx, δy]ᵀ
 * @endverbatim
 * Measurement function:
 * @verbatim
 *   h(x, l) = [ r ] = [ ‖δ‖                        ]
 *             [ φ ]   [ ssa(atan2(δy, δx) - ψ_s)   ]
 * @endverbatim
 * Landmark Jacobian (world frame):
 * @verbatim
 *   H_l = ∂h/∂l = [  δx/r     δy/r  ]
 *                 [ -δy/r²    δx/r² ]
 * @endverbatim
 * Pose Jacobian by the chain rule through the (constant) mount:
 * @verbatim
 *   H_x = ∂h/∂T_ws · ∂T_ws/∂T_wb = ∂h/∂T_ws · Ad(T_bs⁻¹)
 * @endverbatim
 * With an identity mount this reduces to the standard EKF-SLAM range-bearing model.
 * Shared by RangeBearingModel and RangeBearingFactor so both use identical maths.
 *
 * @param T_wb   Body pose in the world.
 * @param T_bs   Sensor mount (sensor pose in the body frame).
 * @param l      Landmark position in the world.
 * @param H_pose If non-null, receives H_x (2 x 3), tangent space at T_wb.
 * @param H_lm   If non-null, receives H_l (2 x 2).
 * @return [r, φ]ᵀ.
 */
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

/**
 * @brief Predicted [range, bearing]  ẑ = h(x, l)  through this model's mount.
 *
 * Thin dynamic-size wrapper around predictRangeBearing(); see there for h, H_x and H_l.
 *
 * @param T_wb   Body pose in the world.
 * @param l      Landmark position in the world.
 * @param H_pose If non-null, receives H_x (2 x 3).
 * @param H_lm   If non-null, receives H_l (2 x 2).
 * @return ẑ = [r̂, φ̂]ᵀ.
 */
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

/**
 * @brief Innovation for a [range, bearing] measurement.
 *
 * @verbatim
 *   ν = [ r - r̂      ]
 *       [ ssa(φ - φ̂) ]
 * @endverbatim
 *
 * @param z     Measurement [r, φ]ᵀ.
 * @param z_hat Prediction [r̂, φ̂]ᵀ.
 * @return ν.
 */
Eigen::VectorXd RangeBearingModel::residual(const Eigen::VectorXd& z, const Eigen::VectorXd& z_hat) const {
    Eigen::VectorXd r = z - z_hat;
    r(1) = math::wrapAngle(r(1));
    return r;
}

/**
 * @brief Inverse measurement: landmark position from one [range, bearing] measurement.
 *
 * With sensor pose T_ws = T_wb ⊕ T_bs (position p_s, heading ψ_s):
 * @verbatim
 *   l = h⁻¹(x, z) = p_s + r · [ cos(ψ_s + φ) ]
 *                             [ sin(ψ_s + φ) ]
 * @endverbatim
 *
 * @param T_wb Body pose in the world.
 * @param z    Measurement [r, φ]ᵀ.
 * @return World landmark position, or std::nullopt if r ≤ 0 (or NaN).
 */
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

/**
 * @brief Is the predicted landmark inside the sensor's field of view?
 *
 * r̂ < r_max  and  |φ̂| < φ_max  (see FovGate::contains()).
 *
 * @param z_hat Predicted measurement [r̂, φ̂]ᵀ.
 * @param gate  Field-of-view gate.
 * @return True if the landmark belongs to the local map.
 */
bool RangeBearingModel::isVisible(const Eigen::VectorXd& z_hat, const FovGate& gate) const {
    return gate.contains(z_hat(0), z_hat(1));
}

/**
 * @brief Build a RangeBearingFactor between pose @p x and landmark @p l.
 *
 * Adds this term to the MAP cost, with R ordered [range, bearing] like z:
 * @verbatim
 *   ½ ‖h(x, l) - z‖²_R,     e.g.  R = diag(σ_r², σ_φ²)
 * @endverbatim
 *
 * @param x     Key of the pose variable.
 * @param l     Key of the landmark variable.
 * @param z     Measurement [r, φ]ᵀ.
 * @param noise 2-D noise model for R, ordered [range, bearing].
 * @return The factor, with this model's mount.
 * @throws std::invalid_argument if @p z or @p noise is not 2-D, or @p noise is null.
 */
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

/**
 * @brief Construct a range-bearing factor.
 *
 * @param x     Key of the pose variable x.
 * @param l     Key of the landmark variable l.
 * @param z     Measurement [r, φ]ᵀ.
 * @param noise Noise model for R, ordered [range, bearing].
 * @param T_bs  Sensor mount (identity = sensor at the body origin).
 */
RangeBearingFactor::RangeBearingFactor(gtsam::Key x, gtsam::Key l, const Eigen::Vector2d& z,
                                       const gtsam::SharedNoiseModel& noise, const gtsam::Pose2& T_bs)
    : Base(noise, x, l), z_(z), T_bs_(T_bs) {}

/**
 * @brief Unwhitened factor error and its Jacobians, called by the optimizer.
 *
 * @verbatim
 *   e(x, l) = [ r̂ - r      ]      with  ẑ = h(x, l)
 *             [ ssa(φ̂ - φ) ]
 *
 *   ∂e/∂x = H_x,   ∂e/∂l = H_l
 * @endverbatim
 * Note the sign: GTSAM uses prediction minus measurement, the opposite of the
 * innovation ν in RangeBearingModel::residual(). GTSAM then whitens e with R and
 * Gauss-Newton / Levenberg-Marquardt linearise with H_x and H_l.
 *
 * @param T_wb   Current estimate of the body pose.
 * @param l      Current estimate of the landmark.
 * @param H_pose If set, receives H_x (2 x 3).
 * @param H_lm   If set, receives H_l (2 x 2).
 * @return e (2-D).
 */
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

/**
 * @brief Deep copy of the factor (needed by GTSAM, e.g. by ISAM2).
 * @return A new RangeBearingFactor equal to this one.
 */
gtsam::NonlinearFactor::shared_ptr RangeBearingFactor::clone() const {
    return boost::make_shared<RangeBearingFactor>(*this);
}

}  // namespace fslam
