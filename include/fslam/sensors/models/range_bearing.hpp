#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include "fslam/sensors/imeasurement_model.hpp"

namespace fslam {

// Prediction of a [range, bearing] measurement from body pose T_wb through the
// mount T_bs, with Jacobians w.r.t. the body pose and the landmark.
// Shared by RangeBearingModel and RangeBearingFactor so both use identical maths.
Eigen::Vector2d predictRangeBearing(const gtsam::Pose2& T_wb, const gtsam::Pose2& T_bs,
                                    const gtsam::Point2& l,
                                    Eigen::Matrix<double, 2, 3>* H_pose,
                                    Eigen::Matrix<double, 2, 2>* H_lm);

// Measurement z = [range, bearing] (framework order).
// Port of GA2 factor_graph.predict_measurement / inverse_measurement.
class RangeBearingModel : public IMeasurementModel {
public:
    std::string name() const override { return "range_bearing"; }
    int dim() const override { return 2; }

    Eigen::VectorXd predict(const gtsam::Pose2& T_wb, const gtsam::Point2& l,
                            Eigen::MatrixXd* H_pose = nullptr,
                            Eigen::MatrixXd* H_lm = nullptr) const override;

    Eigen::VectorXd residual(const Eigen::VectorXd& z, const Eigen::VectorXd& z_hat) const override;

    std::optional<gtsam::Point2> inverse(const gtsam::Pose2& T_wb, const Eigen::VectorXd& z) const override;

    bool isVisible(const Eigen::VectorXd& z_hat, const FovGate& gate) const override;

    // Returns a RangeBearingFactor: error and noise both ordered [range, bearing].
    gtsam::NonlinearFactor::shared_ptr makeFactor(gtsam::Key x, gtsam::Key l,
                                                  const Eigen::VectorXd& z,
                                                  const gtsam::SharedNoiseModel& noise) const override;
};

// Range-bearing factor in framework order [range, bearing], with a sensor mount.
// Equivalent to gtsam::BearingRangeFactor<Pose2, Point2> (which is ordered
// [bearing, range] and has no mount) when the mount is the identity.
// error = h(x, l) - z, bearing component wrapped to [-pi, pi].
class RangeBearingFactor : public gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Point2> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Point2>;

    RangeBearingFactor(gtsam::Key x, gtsam::Key l, const Eigen::Vector2d& z,
                       const gtsam::SharedNoiseModel& noise,
                       const gtsam::Pose2& T_bs = gtsam::Pose2());

    using Base::evaluateError;
    gtsam::Vector evaluateError(const gtsam::Pose2& T_wb, const gtsam::Point2& l,
                                boost::optional<gtsam::Matrix&> H_pose = boost::none,
                                boost::optional<gtsam::Matrix&> H_lm = boost::none) const override;

    gtsam::NonlinearFactor::shared_ptr clone() const override;

    /// @brief Measurement z = [r, φ]ᵀ this factor was built from.
    const Eigen::Vector2d& measured() const { return z_; }
    /// @brief Sensor mount T_bs used in the prediction.
    const gtsam::Pose2& mount() const { return T_bs_; }

private:
    Eigen::Vector2d z_;
    gtsam::Pose2 T_bs_;
};

}  // namespace fslam
