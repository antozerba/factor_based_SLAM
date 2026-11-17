#include "fslam/sensors/preintegrator.hpp"

/*
 * Odometry (wheel encoders, steering) comes in fast, maybe 50–100 Hz.
 * You don't want a graph pose for every sample: the graph would get huge and optimization slow.
 * Instead you only add a pose X(k) at keyframes, e.g. whenever a laser scan arrives.
 *
 */

namespace fslam {

/**
 * @brief Start a new preintegration interval at time @p t.
 *
 * Sets the accumulated increment to the identity and its covariance to zero:
 * @verbatim
 *   ΔT_0 = I  (identity in SE(2))
 *   P_0  = 0  (3x3)
 * @endverbatim
 * Relative to the keyframe itself the robot has not moved, and that is known
 * exactly, so the initial uncertainty is zero. This is the same as starting an
 * EKF prediction chain from a perfectly known state.
 *
 * @param t Start time t0 of the interval [s]; also used as the end time
 *          until the first increment arrives.
 */
void Preintegrator::reset(Timestamp t) {
    delta_ = gtsam::Pose2();
    cov_.setZero();
    t0_ = t;
    t1_ = t;
    count_ = 0;
    started_ = true;
}

/**
 * @brief Add one odometry increment to the running relative motion.
 *
 * Mean: the increment u_k is expressed in the body frame at the end of the
 * previous increment, so it is chained on by pose composition, not summed:
 * @verbatim
 *   ΔT_{k+1} = ΔT_k ⊕ u_k
 * @endverbatim
 *
 * Covariance: first-order (EKF-style) propagation through the composition,
 * i.e. the covariance prediction step:
 * @verbatim
 *   P_{k+1} = F P_k Fᵀ + G Q_k Gᵀ
 *
 *   F = ∂(ΔT_k ⊕ u_k) / ∂ΔT_k = Ad(u_k⁻¹)
 *   G = ∂(ΔT_k ⊕ u_k) / ∂u_k  = I
 * @endverbatim
 * (F and G as given by GTSAM's right / body-frame perturbations.)
 * F rotates the existing uncertainty into the new body frame, which is how heading
 * error turns into lateral position error and gives P its off-diagonal
 * (x–yaw, y–yaw) terms.
 *
 * @param inc     Pose increment u_k in the current body frame.
 * @param inc_cov Covariance Q_k of the increment, tangent space, ordered [x, y, yaw].
 * @param t1      End time of this increment [s]; becomes the interval's end time.
 */
void Preintegrator::integrate(const gtsam::Pose2& inc, const Eigen::Matrix3d& inc_cov, Timestamp t1) {
    gtsam::Matrix3 H1;  // d(delta * inc) / d delta
    gtsam::Matrix3 H2;  // d(delta * inc) / d inc
    delta_ = delta_.compose(inc, H1, H2);
    cov_ = H1 * cov_ * H1.transpose() + H2 * inc_cov * H2.transpose();
    t1_ = t1;
    ++count_;
}

/**
 * @brief Run a motion model on one raw odometry sample, then integrate the result.
 *
 * The motion model maps the raw input to a pose increment and its covariance:
 * @verbatim
 *   u_k = f(u, Δt)     (model.relativePose)
 *   Q_k = Q(u, Δt)     (model.covariance)
 * @endverbatim
 * and these are passed to integrate(const gtsam::Pose2&, const Eigen::Matrix3d&, Timestamp).
 * This keeps the preintegrator independent of the vehicle kinematics.
 *
 * If reset() has never been called, the interval starts at the sample's t0.
 *
 * @param model Motion model that interprets @p s (e.g. AckermannBicycleModel).
 * @param s     Raw odometry sample covering [t0, t1].
 */
void Preintegrator::integrate(const IMotionModel& model, const MotionSample& s) {
    if (!started_) {
        reset(s.t0);  // reset() never called: the interval starts at the first sample
    }
    integrate(model.relativePose(s), model.covariance(s), s.t1);
}

/**
 * @brief Package the accumulated motion for the factor graph.
 *
 * The result becomes the odometry factor between consecutive keyframe poses:
 * @verbatim
 *   e(x_i, x_j) = Log( ΔT⁻¹ · (x_i⁻¹ · x_j) ),    e ~ N(0, P)
 * @endverbatim
 * i.e. a gtsam::BetweenFactor<Pose2> whose noise model is built from P with
 * NoiseModelFactory::fromCovariance().
 *
 * @return Relative motion (ΔT, P, t0, t1) since the last reset().
 */
RelativeMotion Preintegrator::result() const {
    RelativeMotion m;
    m.delta = delta_;
    m.covariance = cov_;
    m.t0 = t0_;
    m.t1 = t1_;
    return m;
}

}  // namespace fslam
