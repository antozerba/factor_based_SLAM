//
// Created by anto on 15/11/26.
//

#ifndef FACTOR_BASED_SLAM_NOISE_MODEL_FACTORY_HPP
#define FACTOR_BASED_SLAM_NOISE_MODEL_FACTORY_HPP

#include <string>

#include <Eigen/Core>
#include <gtsam/linear/NoiseModel.h>

namespace fslam {

    // Robust kernel applied on top of a Gaussian noise model.
    // type: "none" | "huber" | "cauchy" | "tukey" | "dcs"
    // k:    kernel threshold, in units of whitened residual (standard deviations).
    struct RobustKernelConfig {
        std::string type = "none";
        double k = 1.345;   // GA2 default Huber threshold
    };

    // Builds GTSAM noise models from plain numbers. No config/YAML knowledge here:
    // callers (sensors, GraphBuilder, runner) pass in their own values.
    // Replaces GA2 factor_graph.prior_noise_model and bearing_range_noise_model.
    class NoiseModelFactory {
    public:
        // Independent axes: sigmas = standard deviations, in the factor's own ordering.
        static gtsam::SharedNoiseModel fromSigmas(const Eigen::VectorXd& sigmas);

        // Full (possibly correlated) covariance, e.g. preintegrated odometry.
        static gtsam::SharedNoiseModel fromCovariance(const Eigen::MatrixXd& covariance);

        // Wrap `base` in a robust kernel. type "none" returns `base` unchanged.
        // Throws std::invalid_argument for an unknown type or k <= 0.
        static gtsam::SharedNoiseModel withRobust(const gtsam::SharedNoiseModel& base,
                                                  const RobustKernelConfig& kernel);
    };

}  // namespace fslam

#endif //FACTOR_BASED_SLAM_NOISE_MODEL_FACTORY_HPP
