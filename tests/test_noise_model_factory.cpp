//
// Created by anto on 15/11/26.
//
#include <gtest/gtest.h>
#include <boost/pointer_cast.hpp>

#include "fslam/core/noise_model_factory.hpp"

using fslam::NoiseModelFactory;
using fslam::RobustKernelConfig;
namespace nm = gtsam::noiseModel;

TEST(NoiseModelFactory, FromSigmasKeepsSigmas) {
    const Eigen::Vector3d s(0.05, 0.05, 0.01);
    auto model = NoiseModelFactory::fromSigmas(s);

    auto diag = boost::dynamic_pointer_cast<nm::Diagonal>(model);
    ASSERT_TRUE(diag) << "expected a Diagonal model";
    EXPECT_TRUE(diag->sigmas().isApprox(s));
    EXPECT_EQ(model->dim(), 3u);
}

TEST(NoiseModelFactory, FromCovarianceKeepsCorrelations) {
    Eigen::Matrix3d C;
    C << 0.04,  0.01, 0.0,
         0.01,  0.09, 0.002,
         0.0,   0.002, 0.01;
    auto model = NoiseModelFactory::fromCovariance(C);

    auto gauss = boost::dynamic_pointer_cast<nm::Gaussian>(model);
    ASSERT_TRUE(gauss);
    EXPECT_TRUE(gauss->covariance().isApprox(C, 1e-12));
}

TEST(NoiseModelFactory, WhitenedErrorIsMahalanobis) {
    // ||whiten(e)||^2 must equal e^T C^-1 e: the quantity the optimizer minimizes.
    Eigen::Matrix3d C;
    C << 0.04, 0.01, 0.0,
         0.01, 0.09, 0.0,
         0.0,  0.0,  0.01;
    const Eigen::Vector3d e(0.1, -0.2, 0.05);
    auto model = NoiseModelFactory::fromCovariance(C);

    EXPECT_NEAR(model->whiten(e).squaredNorm(), e.dot(C.inverse() * e), 1e-10);
}

TEST(NoiseModelFactory, NoneReturnsBaseUnchanged) {
    auto base = NoiseModelFactory::fromSigmas(Eigen::Vector2d(0.017, 0.2));
    auto model = NoiseModelFactory::withRobust(base, RobustKernelConfig{"none", 1.0});
    EXPECT_EQ(model, base);   // same object, not a copy
}

TEST(NoiseModelFactory, HuberWrapsBaseWithRightThreshold) {
    auto base = NoiseModelFactory::fromSigmas(Eigen::Vector2d(0.017, 0.2));
    auto model = NoiseModelFactory::withRobust(base, RobustKernelConfig{"huber", 1.345});

    auto robust = boost::dynamic_pointer_cast<nm::Robust>(model);
    ASSERT_TRUE(robust) << "expected a Robust model";
    EXPECT_EQ(robust->noise(), base);

    auto huber = boost::dynamic_pointer_cast<nm::mEstimator::Huber>(robust->robust());
    ASSERT_TRUE(huber) << "expected a Huber estimator";
    EXPECT_DOUBLE_EQ(huber->modelParameter(), 1.345);
}

TEST(NoiseModelFactory, RejectsBadInput) {
    auto base = NoiseModelFactory::fromSigmas(Eigen::Vector2d(0.1, 0.1));
    EXPECT_THROW(NoiseModelFactory::withRobust(base, {"banana", 1.0}), std::invalid_argument);
    EXPECT_THROW(NoiseModelFactory::withRobust(base, {"huber", 0.0}), std::invalid_argument);
    EXPECT_THROW(NoiseModelFactory::fromSigmas(Eigen::Vector2d(0.1, -1.0)), std::invalid_argument);
    EXPECT_THROW(NoiseModelFactory::fromCovariance(Eigen::MatrixXd(2, 3)), std::invalid_argument);
}
