#include "fslam/sensors/motion/direct_relative_pose.hpp"

#include <stdexcept>

namespace fslam {

gtsam::Pose2 DirectRelativePoseModel::relativePose(const MotionSample& s) const {
    if (s.u.size() != inputDim()) {
        throw std::invalid_argument("DirectRelativePoseModel: u must be [dx, dy, dyaw]");
    }
    return gtsam::Pose2(s.u(0), s.u(1), s.u(2));
}

}  // namespace fslam
