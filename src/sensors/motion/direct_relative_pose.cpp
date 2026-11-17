#include "fslam/sensors/motion/direct_relative_pose.hpp"

#include <stdexcept>

namespace fslam {

/**
 * @brief Pose increment given directly by the dataset.
 *
 * @verbatim
 *   u_k = (Δx, Δy, Δψ) ∈ SE(2)
 * @endverbatim
 * i.e. the input already is the body-frame increment; no kinematics are applied.
 *
 * @param s Sample with u = [Δx, Δy, Δψ].
 * @return u_k.
 * @throws std::invalid_argument if u is not 3-D.
 */
gtsam::Pose2 DirectRelativePoseModel::relativePose(const MotionSample& s) const {
    if (s.u.size() != inputDim()) {
        throw std::invalid_argument("DirectRelativePoseModel: u must be [dx, dy, dyaw]");
    }
    return gtsam::Pose2(s.u(0), s.u(1), s.u(2));
}

}  // namespace fslam
