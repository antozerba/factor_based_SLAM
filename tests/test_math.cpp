#include <cmath>
#include <gtest/gtest.h>
#include "fslam/core/math.hpp"


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
