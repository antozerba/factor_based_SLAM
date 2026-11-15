// AckermannBicycleModel, DirectRelativePoseModel, Preintegrator.
// Ports GA2 tests/test_preprocessing.py (relative_pose, preintegrate).

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Eigenvalues>

#include "fslam/sensors/motion/ackermann_bicycle.hpp"
#include "fslam/sensors/motion/direct_relative_pose.hpp"
#include "fslam/sensors/preintegrator.hpp"

using fslam::AckermannBicycleModel;
using fslam::DirectRelativePoseModel;
using fslam::MotionSample;
using fslam::Preintegrator;
using gtsam::Pose2;

namespace {

constexpr double kL = 2.83;
constexpr double kH = 0.76;

AckermannBicycleModel victoriaCar() {
    AckermannBicycleModel::Params p;
    p.wheelbase = kL;
    p.encoder_offset = kH;
    return AckermannBicycleModel(p);
}

MotionSample sample(double v, double steer, double dt) {
    return MotionSample{0.0, dt, Eigen::Vector2d(v, steer)};
}

void expectPose(const Pose2& p, double x, double y, double th, double tol) {
    EXPECT_NEAR(p.x(), x, tol);
    EXPECT_NEAR(p.y(), y, tol);
    EXPECT_NEAR(p.theta(), th, tol);
}

}  // namespace

// ---------------------------------------------------------------------------
// AckermannBicycleModel  (GA2 Task 1a)
// ---------------------------------------------------------------------------

TEST(AckermannBicycle, StraightMotion) {
    expectPose(victoriaCar().relativePose(sample(2.0, 0.0, 0.5)), 1.0, 0.0, 0.0, 1e-12);
}

TEST(AckermannBicycle, AppliesEncoderOffsetCorrection) {
    const double v = 3.0, steer = 0.20, dt = 0.4;
    const double v_body = v / (1.0 - (kH / kL) * std::tan(steer));
    const double expected_yaw = v_body / kL * std::tan(steer) * dt;

    EXPECT_NEAR(victoriaCar().relativePose(sample(v, steer, dt)).theta(), expected_yaw, 1e-12);
}

TEST(AckermannBicycle, UsesItsOwnGeometry) {
    AckermannBicycleModel::Params p;
    p.wheelbase = 2.0;
    p.encoder_offset = 0.0;  // encoder on the centre line: no correction
    const AckermannBicycleModel centred(p);
    const double v = 3.0, steer = 0.20, dt = 0.4;

    EXPECT_NEAR(centred.relativePose(sample(v, steer, dt)).theta(), v * std::tan(steer) / 2.0 * dt, 1e-12);
}

TEST(AckermannBicycle, FollowsAnArcNotAStraightLine) {
    // Closed-form unicycle with constant v, omega (GA2 used a numerical ODE solve).
    const double v = 4.0, steer = 0.35, dt = 1.0;
    const double v_body = v / (1.0 - (kH / kL) * std::tan(steer));
    const double w = v_body / kL * std::tan(steer);
    const double th = w * dt;

    const Pose2 pose = victoriaCar().relativePose(sample(v, steer, dt));

    expectPose(pose, v_body / w * std::sin(th), v_body / w * (1.0 - std::cos(th)), th, 1e-9);
    EXPECT_GT(std::abs(pose.y()), 1e-3) << "an Euler step would have y == 0";
}

TEST(AckermannBicycle, ZeroVelocityIsIdentity) {
    expectPose(victoriaCar().relativePose(sample(0.0, 0.3, 1.0)), 0.0, 0.0, 0.0, 1e-12);
}

TEST(AckermannBicycle, CovarianceScalesWithDt) {
    AckermannBicycleModel::Params p;
    p.Q_per_second = Eigen::Vector3d(0.01, 0.01, 3e-6).asDiagonal();
    const AckermannBicycleModel model(p);

    EXPECT_TRUE(model.covariance(sample(1.0, 0.0, 0.025)).isApprox(p.Q_per_second * 0.025));
}

TEST(AckermannBicycle, RejectsWrongInputSize) {
    MotionSample s{0.0, 1.0, Eigen::Vector3d(1, 2, 3)};
    EXPECT_THROW(victoriaCar().relativePose(s), std::invalid_argument);
}

TEST(DirectRelativePose, ReturnsTheGivenIncrementAndConstantQ) {
    const Eigen::Matrix3d Q = Eigen::Vector3d(0.01, 0.02, 0.003).asDiagonal();
    const DirectRelativePoseModel model(Q);
    const MotionSample s{0.0, 7.0, Eigen::Vector3d(1.0, -0.5, 0.2)};

    expectPose(model.relativePose(s), 1.0, -0.5, 0.2, 1e-15);
    EXPECT_TRUE(model.covariance(s).isApprox(Q));  // not scaled by dt
}

// ---------------------------------------------------------------------------
// Preintegrator  (GA2 Task 1b)
// ---------------------------------------------------------------------------

namespace {

std::pair<Pose2, Eigen::Matrix3d> preintegrate(const std::vector<Pose2>& incs,
                                               const std::vector<Eigen::Matrix3d>& covs) {
    Preintegrator p;
    p.reset(0.0);
    for (std::size_t i = 0; i < incs.size(); ++i) {
        p.integrate(incs[i], covs[i], static_cast<double>(i + 1));
    }
    return {p.delta(), p.covariance()};
}

const Eigen::Matrix3d kCov = Eigen::Vector3d(0.04, 0.01, 0.0009).asDiagonal();

}  // namespace

TEST(Preintegrator, NothingIsIdentity) {
    const auto [pose, cov] = preintegrate({}, {});
    expectPose(pose, 0, 0, 0, 1e-12);
    EXPECT_TRUE(cov.isZero(1e-12));
}

TEST(Preintegrator, CompoundsTheMean) {
    const Pose2 a(1.0, 0.0, M_PI / 2), b(2.0, 0.0, 0.0);
    const auto [pose, cov] = preintegrate({a, b}, {Eigen::Matrix3d::Identity() * 1e-6,
                                                   Eigen::Matrix3d::Identity() * 1e-6});
    const Pose2 expected = a.compose(b);
    expectPose(pose, expected.x(), expected.y(), expected.theta(), 1e-12);
}

TEST(Preintegrator, SingleIncrementKeepsItsCovariance) {
    const auto [pose, cov] = preintegrate({Pose2(1.5, -0.2, 0.3)}, {kCov});
    EXPECT_TRUE(cov.isApprox(kCov, 1e-12));
}

TEST(Preintegrator, TransportsCovarianceThroughSecondIncrement) {
    const Pose2 first(1.0, 0.0, 0.4), second(2.0, 0.5, -0.7);
    const auto [pose, cov] = preintegrate({first, second}, {kCov, Eigen::Matrix3d::Zero()});

    const Eigen::Matrix3d Ad = second.inverse().AdjointMap();
    EXPECT_TRUE(cov.isApprox(Ad * kCov * Ad.transpose(), 1e-10));
}

TEST(Preintegrator, DoesNotSimplySumCovariances) {
    const Pose2 inc(1.0, 0.0, 0.5);
    const auto [pose, cov] = preintegrate({inc, inc}, {kCov, kCov});

    EXPECT_FALSE(cov.isApprox(kCov + kCov, 1e-6));
    EXPECT_TRUE(cov.isApprox(cov.transpose(), 1e-12));
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    EXPECT_GE(eig.eigenvalues().minCoeff(), -1e-12);
}

TEST(Preintegrator, CovarianceGrowsWithMoreIncrements) {
    const Pose2 inc(1.0, 0.0, 0.1);
    const auto [p2, short_cov] = preintegrate(std::vector<Pose2>(2, inc), std::vector<Eigen::Matrix3d>(2, kCov));
    const auto [p5, long_cov] = preintegrate(std::vector<Pose2>(5, inc), std::vector<Eigen::Matrix3d>(5, kCov));
    EXPECT_GT(long_cov.trace(), short_cov.trace());
}

TEST(Preintegrator, ModelOverloadTracksTimeAndResets) {
    const AckermannBicycleModel car = victoriaCar();
    Preintegrator p;
    p.integrate(car, MotionSample{10.0, 10.1, Eigen::Vector2d(2.0, 0.0)});
    p.integrate(car, MotionSample{10.1, 10.3, Eigen::Vector2d(2.0, 0.0)});

    const fslam::RelativeMotion m = p.result();
    EXPECT_EQ(p.count(), 2u);
    EXPECT_DOUBLE_EQ(m.t0, 10.0);
    EXPECT_DOUBLE_EQ(m.t1, 10.3);
    expectPose(m.delta, 0.6, 0.0, 0.0, 1e-12);

    p.reset(10.3);
    EXPECT_TRUE(p.empty());
    expectPose(p.delta(), 0, 0, 0, 0);
    EXPECT_DOUBLE_EQ(p.result().t0, 10.3);
}
