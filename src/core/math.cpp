#include "fslam/core/math.hpp"
#include <cmath>


namespace fslam::math{

    double wrapAngle(double angle)
    {
        return std::remainder(angle, 2.0 * M_PI);
    }
}

