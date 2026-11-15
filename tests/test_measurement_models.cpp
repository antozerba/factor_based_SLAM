// RangeBearingModel and RangeBearingFactor.
// Ports the GA2 predict_measurement / inverse_measurement tests and checks every
// analytic Jacobian against gtsam::numericalDerivative.

#include <gtest/gtest.h>

#include <cmath>
#include <functional>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include "fslam/core/noise_model_factory.hpp"
#include "fslam/sensors/models/range_bearing.hpp"

using fslam::FovGate;
using fslam::RangeBearingFactor;
using fslam::RangeBearingModel;
using fslam::SensorMount;
using gtsam::Point2;
using gtsam::Pose2;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

namespace {

const Pose2 kPose(1.0, 2.0, 0.6);
const Point2 kLandmark(6.0, 4.5);
const Pose2 kMount(3.78, 0.5, 0.05);  // Victoria Park a/b offsets + a small yaw

RangeBearingModel model(const Pose2& mount = Pose2()) {
    RangeBearingModel m;
    m.setMount(SensorMount{mount});
    return m;
}

}  // namespace

// ---------------------------------------------------------------------------
// predict
// ---------------------------------------------------------------------------

TEST(RangeBearingModel, PredictMatchesGeometry) {
    const Eigen::VectorXd z = model().predict(kPose, kLandmark);
    const Point2 d = kLandmark - kPose.translation();

    ASSERT_EQ(z.size(), 2);
    EXPECT_NEAR(z(0), d.norm(), 1e-12);                                      // range first
    EXPECT_NEAR(z(1), std::atan2(d.y(), d.x()) - kPose.theta(), 1e-12);      // then bearing
}

class RangeBearingJacobians : public ::testing::TestWithParam<Pose2> {};

TEST_P(RangeBearingJacobians, MatchNumericalDerivative) {
    const RangeBearingModel m = model(GetParam());
    Eigen::MatrixXd H_pose, H_lm;
    m.predict(kPose, kLandmark, &H_pose, &H_lm);

    std::function<gtsam::Vector2(const Pose2&, const Point2&)> h =
        [&m](const Pose2& x, const Point2& l) { return gtsam::Vector2(m.predict(x, l)); };
    const gtsam::Matrix num_pose = gtsam::numericalDerivative21<gtsam::Vector2, Pose2, Point2>(h, kPose, kLandmark);
    const gtsam::Matrix num_lm = gtsam::numericalDerivative22<gtsam::Vector2, Pose2, Point2>(h, kPose, kLandmark);

    EXPECT_TRUE(H_pose.isApprox(num_pose, 1e-7)) << "analytic\n" << H_pose << "\nnumerical\n" << num_pose;
    EXPECT_TRUE(H_lm.isApprox(num_lm, 1e-7)) << "analytic\n" << H_lm << "\nnumerical\n" << num_lm;
}

INSTANTIATE_TEST_SUITE_P(IdentityAndOffsetMount, RangeBearingJacobians,
                         ::testing::Values(Pose2(), kMount));

TEST(RangeBearingModel, MountShiftsThePrediction) {
    const Eigen::VectorXd z = model(kMount).predict(kPose, kLandmark);
    const Pose2 T_ws = kPose.compose(kMount);
    EXPECT_NEAR(z(0), T_ws.range(kLandmark), 1e-12);
    EXPECT_NEAR(z(1), T_ws.bearing(kLandmark).theta(), 1e-12);
}

// ---------------------------------------------------------------------------
// inverse / residual / visibility
// ---------------------------------------------------------------------------

TEST(RangeBearingModel, InverseRoundTrips) {
    for (const Pose2& mount : {Pose2(), kMount}) {
        const RangeBearingModel m = model(mount);
        const auto l = m.inverse(kPose, m.predict(kPose, kLandmark));
        ASSERT_TRUE(l.has_value());
        EXPECT_TRUE(l->isApprox(kLandmark, 1e-12)) << *l;
    }
}

TEST(RangeBearingModel, InverseOfGa2Example) {
    // Straight ahead at 5 m from a pose facing +y -> landmark is 5 m along +y.
    const auto l = model().inverse(Pose2(1.0, 1.0, M_PI / 2), Eigen::Vector2d(5.0, 0.0));
    ASSERT_TRUE(l);
    EXPECT_TRUE(l->isApprox(Point2(1.0, 6.0), 1e-12));
}

TEST(RangeBearingModel, InverseRejectsNonPositiveRange) {
    EXPECT_FALSE(model().inverse(kPose, Eigen::Vector2d(0.0, 0.3)).has_value());
}

TEST(RangeBearingModel, ResidualWrapsBearingOnly) {
    const Eigen::VectorXd r = model().residual(Eigen::Vector2d(10.0, 3.1), Eigen::Vector2d(9.0, -3.1));
    EXPECT_NEAR(r(0), 1.0, 1e-12);
    EXPECT_NEAR(r(1), 6.2 - 2 * M_PI, 1e-12);  // not 6.2
}

TEST(RangeBearingModel, VisibilityUsesRangeAndFov) {
    const FovGate gate{50.0, 125.0 * M_PI / 180.0};
    const RangeBearingModel m = model();
    EXPECT_TRUE(m.isVisible(Eigen::Vector2d(10.0, 0.5), gate));
    EXPECT_FALSE(m.isVisible(Eigen::Vector2d(60.0, 0.5), gate));   // too far
    EXPECT_FALSE(m.isVisible(Eigen::Vector2d(10.0, 2.5), gate));   // outside FOV
}

// ---------------------------------------------------------------------------
// factor
// ---------------------------------------------------------------------------

TEST(RangeBearingFactor, MatchesGtsamBearingRangeFactorWithIdentityMount) {
    const double sigma_r = 0.2, sigma_b = 1.0 * M_PI / 180.0;
    const Eigen::Vector2d z(7.0, 0.1);  // [range, bearing]

    // Ours: framework order [range, bearing]; GTSAM: [bearing, range].
    auto ours = model().makeFactor(X(1), L(0), z, fslam::NoiseModelFactory::fromSigmas(Eigen::Vector2d(sigma_r, sigma_b)));
    gtsam::BearingRangeFactor<Pose2, Point2> theirs(
        X(1), L(0), gtsam::Rot2::fromAngle(z(1)), z(0),
        fslam::NoiseModelFactory::fromSigmas(Eigen::Vector2d(sigma_b, sigma_r)));

    gtsam::Values v;
    v.insert(X(1), kPose);
    v.insert(L(0), kLandmark);

    EXPECT_NEAR(ours->error(v), theirs.error(v), 1e-10);
}

TEST(RangeBearingFactor, JacobiansMatchNumericalDerivative) {
    const RangeBearingFactor f(X(1), L(0), Eigen::Vector2d(7.0, 0.1),
                               fslam::NoiseModelFactory::fromSigmas(Eigen::Vector2d(0.2, 0.02)), kMount);
    gtsam::Matrix H_pose, H_lm;
    f.evaluateError(kPose, kLandmark, H_pose, H_lm);

    std::function<gtsam::Vector(const Pose2&, const Point2&)> e =
        [&f](const Pose2& x, const Point2& l) { return f.evaluateError(x, l); };
    EXPECT_TRUE(H_pose.isApprox(gtsam::numericalDerivative21<gtsam::Vector, Pose2, Point2>(e, kPose, kLandmark), 1e-7));
    EXPECT_TRUE(H_lm.isApprox(gtsam::numericalDerivative22<gtsam::Vector, Pose2, Point2>(e, kPose, kLandmark), 1e-7));
}

TEST(RangeBearingFactor, ErrorIsZeroAtTheTruth) {
    const RangeBearingModel m = model(kMount);
    const RangeBearingFactor f(X(1), L(0), Eigen::Vector2d(m.predict(kPose, kLandmark)),
                               fslam::NoiseModelFactory::fromSigmas(Eigen::Vector2d(0.2, 0.02)), kMount);
    EXPECT_TRUE(f.evaluateError(kPose, kLandmark).isZero(1e-12));
}

TEST(RangeBearingFactor, MakeFactorRejectsWrongDimensions) {
    auto noise3 = fslam::NoiseModelFactory::fromSigmas(Eigen::Vector3d(1, 1, 1));
    EXPECT_THROW(model().makeFactor(X(0), L(0), Eigen::Vector2d(1, 0), noise3), std::invalid_argument);
}
