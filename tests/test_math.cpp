#include <cmath>
#include <gtest/gtest.h>
#include "fslam/core/math.hpp"
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using fslam::math::assembleJointCovariance;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

using namespace fslam::math;

TEST(WrapAngle, InsideRangeUnchanged){
    EXPECT_DOUBLE_EQ(wrapAngle(0.0), 0.0);
    EXPECT_DOUBLE_EQ(wrapAngle(M_PI), M_PI);
    EXPECT_DOUBLE_EQ(wrapAngle(-M_PI), -M_PI);
    EXPECT_DOUBLE_EQ(wrapAngle(M_PI/2), M_PI/2);
    EXPECT_DOUBLE_EQ(wrapAngle(-M_PI/2), -M_PI/2);
}

TEST(WrapAngle, OutsideRangeWrapped){
    EXPECT_DOUBLE_EQ(wrapAngle(3*M_PI), -M_PI);
    EXPECT_DOUBLE_EQ(wrapAngle(-3*M_PI), M_PI);
    EXPECT_DOUBLE_EQ(wrapAngle(5*M_PI/2), M_PI/2);
    EXPECT_DOUBLE_EQ(wrapAngle(-5*M_PI/2), -M_PI/2);
}

namespace {

// Small SLAM problem: X0 --odom--> X1, X1 observes L0 and L1.
struct TinyProblem {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    TinyProblem() {
        using namespace gtsam;
        auto prior_noise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));
        auto odom_noise  = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
        auto meas_noise  = noiseModel::Diagonal::Sigmas(Vector2(0.05, 0.3));  // [bearing, range]

        graph.addPrior(X(0), Pose2(0, 0, 0), prior_noise);
        graph.emplace_shared<BetweenFactor<Pose2>>(X(0), X(1), Pose2(1, 0, 0.1), odom_noise);
        graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(
            X(1), L(0), Rot2::fromAngle(0.5), 3.0, meas_noise);
        graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(
            X(1), L(1), Rot2::fromAngle(-0.4), 4.0, meas_noise);

        values.insert(X(0), Pose2(0, 0, 0));
        values.insert(X(1), Pose2(1, 0, 0.1));
        values.insert(L(0), Point2(3.3, 1.9));
        values.insert(L(1), Point2(4.6, -1.1));
    }
};

}  // namespace

TEST(AssembleJointCovariance, SizeIsPosePlusTwoPerLandmark) {
    TinyProblem p;
    gtsam::Marginals marginals(p.graph, p.values);
    const gtsam::KeyVector keys{X(1), L(0), L(1)};

    const Eigen::MatrixXd P = assembleJointCovariance(marginals.jointMarginalCovariance(keys), keys);

    EXPECT_EQ(P.rows(), 3 + 2 * 2);
    EXPECT_EQ(P.cols(), 3 + 2 * 2);
}

TEST(AssembleJointCovariance, BlocksMatchGtsamAndFollowKeyOrder) {
    TinyProblem p;
    gtsam::Marginals marginals(p.graph, p.values);
    // Deliberately put L1 before L0: the output must follow THIS order.
    const gtsam::KeyVector keys{X(1), L(1), L(0)};
    const auto jm = marginals.jointMarginalCovariance(keys);

    const Eigen::MatrixXd P = assembleJointCovariance(jm, keys);

    const std::vector<Eigen::Index> off{0, 3, 5, 7};
    for (std::size_t i = 0; i < keys.size(); ++i) {
        for (std::size_t j = 0; j < keys.size(); ++j) {
            const gtsam::Matrix expected = jm.at(keys[i], keys[j]);
            const Eigen::MatrixXd actual =
                P.block(off[i], off[j], off[i + 1] - off[i], off[j + 1] - off[j]);
            EXPECT_TRUE(actual.isApprox(expected, 1e-12))
                << "block (" << i << "," << j << ")\nexpected\n" << expected << "\nactual\n" << actual;
        }
    }
    // L1's own covariance must sit at rows/cols 3..4
    EXPECT_TRUE(P.block(3, 3, 2, 2).isApprox(marginals.marginalCovariance(L(1)), 1e-9));
}

TEST(AssembleJointCovariance, IsSymmetricPositiveDefinite) {
    TinyProblem p;
    gtsam::Marginals marginals(p.graph, p.values);
    const gtsam::KeyVector keys{X(1), L(0), L(1)};

    const Eigen::MatrixXd P = assembleJointCovariance(marginals.jointMarginalCovariance(keys), keys);

    EXPECT_TRUE(P.isApprox(P.transpose(), 1e-12));
    Eigen::LLT<Eigen::MatrixXd> llt(P);   // Cholesky succeeds only for SPD matrices
    EXPECT_EQ(llt.info(), Eigen::Success);
}