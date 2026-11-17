#include "fslam/sensors/motion/ackermann_bicycle.hpp"

#include <cmath>
#include <stdexcept>

namespace fslam {

/**
 * @brief Construct the model from vehicle geometry and process noise.
 * @param p Wheelbase L, encoder offset H and continuous-time noise Q_c.
 * @throws std::invalid_argument if L ≤ 0.
 */
AckermannBicycleModel::AckermannBicycleModel(const Params& p) : p_(p) {
    if (p_.wheelbase <= 0.0) {
        throw std::invalid_argument("AckermannBicycleModel: wheelbase must be > 0");
    }
}

/**
 * @brief Body-frame pose increment of the kinematic bicycle model over one sample.
 *
 * The wheel encoder sits a distance H off the centre line, so when turning it
 * measures a different speed than the centre of the rear axle. With steering angle
 * α and wheelbase L:
 * @verbatim
 *   v_c = v_e / (1 - (H/L)·tan α)
 *   ω   = v_c · tan α / L
 * @endverbatim
 * Assuming a constant body twist ξ = [v_c, 0, ω]ᵀ over Δt, the increment is the
 * exact (arc) solution, not an Euler step:
 * @verbatim
 *   u_k = Exp(ξ·Δt) = [ (v_c/ω)·sin(ωΔt)       ]
 *                     [ (v_c/ω)·(1 - cos(ωΔt)) ]
 *                     [ ωΔt                    ]
 *
 *   ω → 0:  u_k = [v_c·Δt, 0, 0]ᵀ
 * @endverbatim
 *
 * @param s Sample with u = [v_e (m/s), α (rad)].
 * @return Pose increment u_k in the body frame at t0.
 * @throws std::invalid_argument if u is not 2-D.
 */
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

/**
 * @brief Discrete process noise of one increment.
 *
 * First-order discretisation of continuous-time white process noise:
 * @verbatim
 *   Q_k = ∫₀^Δt e^(Aτ) G Q_c Gᵀ e^(Aᵀτ) dτ  ≈  Q_c·Δt
 * @endverbatim
 * valid for small Δt. The uncertainty therefore grows linearly with time,
 * whatever the number of samples.
 *
 * @param s Sample; only Δt is used.
 * @return Q_k = Q_c·Δt, ordered [x, y, yaw].
 */
Eigen::Matrix3d AckermannBicycleModel::covariance(const MotionSample& s) const {
    return p_.Q_per_second * s.dt();
}

}  // namespace fslam
