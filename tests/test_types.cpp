//
// Created by anto on 15/11/26.
//
#include <gtest/gtest.h>
#include <fslam/core/types.hpp>
using namespace fslam;

TEST(Types, MotionSampleDt) {
    fslam::MotionSample s{1.0, 1.5, Eigen::Vector2d(2.0, 0.1)};
    EXPECT_DOUBLE_EQ(s.dt(), 0.5);
}

TEST(Types, RawSensorFrameHoldsScan) {
    fslam::RawSensorFrame f;
    f.payload = fslam::LaserScan{};
    EXPECT_TRUE(std::holds_alternative<fslam::LaserScan>(f.payload));
}