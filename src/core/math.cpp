#include "fslam/core/math.hpp"
#include <cmath>



namespace fslam::math{

    /**
     * @brief Smallest signed angle: wrap an angle to \f$ [-\pi, \pi] \f$.
     *
     * @param angle Angle
     * @return Equivalent angle
     */
    double wrapAngle(double angle)
    {
        return std::remainder(angle, 2.0 * M_PI);
    }



    /**
     * @brief Assemble a dense joint covariance P  matrix from a GTSAM JointMarginal.
     * @param joint_marginal Joint marginal over (at least) @p keys, from gtsam::Marginals.
     * @param keys           Variables to include, in the desired block order.
     * @return Dense symmetric matrix  P
     */
    Eigen::MatrixXd assembleJointCovariance(const gtsam::JointMarginal& joint_marginal,
                                        const gtsam::KeyVector& keys) {
        const std::size_t n = keys.size();

        // offsets[i] = row/column where the block of keys[i] starts; offsets[n] = total size.
        std::vector<Eigen::Index> offsets(n + 1, 0);
        for (std::size_t i = 0; i < n; ++i) {
            const Eigen::Index dim = joint_marginal.at(keys[i], keys[i]).rows();
            offsets[i + 1] = offsets[i] + dim;
        }

        const Eigen::Index total = offsets[n];
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(total, total);

        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j) {
                const gtsam::Matrix block = joint_marginal.at(keys[i], keys[j]);
                P.block(offsets[i], offsets[j], block.rows(), block.cols()) = block;
            }
        }
        return P;
    }
}

