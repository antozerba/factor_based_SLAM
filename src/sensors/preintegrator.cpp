#include "fslam/sensors/preintegrator.hpp"

namespace fslam {

void Preintegrator::reset(Timestamp t) {
    delta_ = gtsam::Pose2();
    cov_.setZero();
    t0_ = t;
    t1_ = t;
    count_ = 0;
    started_ = true;
}

void Preintegrator::integrate(const gtsam::Pose2& inc, const Eigen::Matrix3d& inc_cov, Timestamp t1) {
    gtsam::Matrix3 H1;  // d(delta * inc) / d delta
    gtsam::Matrix3 H2;  // d(delta * inc) / d inc
    delta_ = delta_.compose(inc, H1, H2);
    cov_ = H1 * cov_ * H1.transpose() + H2 * inc_cov * H2.transpose();
    t1_ = t1;
    ++count_;
}

void Preintegrator::integrate(const IMotionModel& model, const MotionSample& s) {
    if (!started_) {
        reset(s.t0);  // reset() never called: the interval starts at the first sample
    }
    integrate(model.relativePose(s), model.covariance(s), s.t1);
}

RelativeMotion Preintegrator::result() const {
    RelativeMotion m;
    m.delta = delta_;
    m.covariance = cov_;
    m.t0 = t0_;
    m.t1 = t1_;
    return m;
}

}  // namespace fslam
