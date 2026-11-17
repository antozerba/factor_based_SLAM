//
// Created by anto on 15/11/26.
//
#include "fslam/core/noise_model_factory.hpp"

#include <stdexcept>

namespace fslam {

    namespace nm = gtsam::noiseModel;

    /**
     * @brief Diagonal Gaussian noise model from per-axis standard deviations.
     * @param sigmas Standard deviations \f$ \sigma_i > 0 \f$, in the factor's own error order.
     * @return gtsam::noiseModel::Diagonal.
     * @throws std::invalid_argument if @p sigmas is empty or any \f$ \sigma_i \le 0 \f$.
     */
    gtsam::SharedNoiseModel NoiseModelFactory::fromSigmas(const Eigen::VectorXd& sigmas) {
        if (sigmas.size() == 0 || (sigmas.array() <= 0.0).any()) {
            throw std::invalid_argument("NoiseModelFactory::fromSigmas: sigmas must be non-empty and > 0");
        }
        return nm::Diagonal::Sigmas(sigmas);
    }

    /**
     * @brief Full (possibly correlated) Gaussian noise model from a covariance matrix.
     * @param covariance Symmetric positive definite \f$ \Sigma \f$ (m x m).
     * @return gtsam::noiseModel::Gaussian (or Diagonal).
     * @throws std::invalid_argument if @p covariance is empty or not square.
     * @note Positive definiteness is not checked here; GTSAM's Cholesky fails if it does not hold.
     */
    gtsam::SharedNoiseModel NoiseModelFactory::fromCovariance(const Eigen::MatrixXd& covariance) {
        if (covariance.rows() == 0 || covariance.rows() != covariance.cols()) {
            throw std::invalid_argument("NoiseModelFactory::fromCovariance: covariance must be square and non-empty");
        }
        // smart = true (default): returns a Diagonal model if C happens to be diagonal.
        return nm::Gaussian::Covariance(covariance);
    }

    /**
     * @brief Wrap a Gaussian noise model in a robust M-estimator kernel.
     * @param base   Gaussian noise model to wrap.
     * @param kernel Kernel type ("none", "huber", "cauchy", "tukey", "dcs") and threshold \f$ k \f$.
     * @return @p base unchanged if the type is "none", otherwise a gtsam::noiseModel::Robust.
     * @throws std::invalid_argument for an unknown type or \f$ k \le 0 \f$.
     */
    gtsam::SharedNoiseModel NoiseModelFactory::withRobust(const gtsam::SharedNoiseModel& base,
                                                          const RobustKernelConfig& kernel) {
        if (kernel.type == "none") {
            return base;
        }
        if (kernel.k <= 0.0) {
            throw std::invalid_argument("NoiseModelFactory::withRobust: k must be > 0");
        }

        nm::mEstimator::Base::shared_ptr estimator;
        if (kernel.type == "huber") {
            estimator = nm::mEstimator::Huber::Create(kernel.k);
        } else if (kernel.type == "cauchy") {
            estimator = nm::mEstimator::Cauchy::Create(kernel.k);
        } else if (kernel.type == "tukey") {
            estimator = nm::mEstimator::Tukey::Create(kernel.k);
        } else if (kernel.type == "dcs") {
            estimator = nm::mEstimator::DCS::Create(kernel.k);
        } else {
            throw std::invalid_argument("NoiseModelFactory::withRobust: unknown kernel type '" +
                                        kernel.type + "'");
        }
        return nm::Robust::Create(estimator, base);
    }

}  // namespace fslam
