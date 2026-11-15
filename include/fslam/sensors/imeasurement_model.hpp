#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace fslam {

// Where the sensor sits on the robot: T_body_sensor maps sensor-frame points to the body frame.
struct SensorMount {
    gtsam::Pose2 T_body_sensor;   // identity = sensor at the body origin (GA2 behaviour)
};

// Region in which a landmark is expected to be visible (GA2 range_local / bearing_local).
struct FovGate {
    double max_range = std::numeric_limits<double>::infinity();
    double half_fov_rad = M_PI;

    bool contains(double range, double bearing) const {
        return range < max_range && std::abs(bearing) < half_fov_rad;
    }
};

// The seam that makes the framework sensor-agnostic: everything that in GA2
// assumed [range, bearing] (predict_measurement, inverse_measurement, ssa on
// index 1, BearingRangeFactor2D) goes through this interface.
// Poses are body poses T_wb; the model applies its mount internally, and all
// Jacobians are with respect to the BODY pose (tangent space at T_wb).
class IMeasurementModel {
public:
    virtual ~IMeasurementModel() = default;

    virtual std::string name() const = 0;

    // Measurement dimension m (2 for range-bearing, 1 for bearing-only, ...).
    virtual int dim() const = 0;

    // Predicted measurement of world landmark l from body pose T_wb.
    // If non-null, H_pose (m x 3) and H_lm (m x 2) receive the Jacobians.
    virtual Eigen::VectorXd predict(const gtsam::Pose2& T_wb, const gtsam::Point2& l,
                                    Eigen::MatrixXd* H_pose = nullptr,
                                    Eigen::MatrixXd* H_lm = nullptr) const = 0;

    // z - z_hat, with angular components wrapped.
    virtual Eigen::VectorXd residual(const Eigen::VectorXd& z, const Eigen::VectorXd& z_hat) const = 0;

    // World landmark position from one measurement, or nullopt if the model
    // cannot invert it (e.g. bearing-only needs delayed initialisation).
    virtual std::optional<gtsam::Point2> inverse(const gtsam::Pose2& T_wb, const Eigen::VectorXd& z) const = 0;

    // Is a landmark with predicted measurement z_hat inside the gate (local map selection)?
    virtual bool isVisible(const Eigen::VectorXd& z_hat, const FovGate& gate) const = 0;

    // Factor between pose x and landmark l for measurement z. The noise model is
    // in the SAME component order as z (no reordering needed by the caller).
    virtual gtsam::NonlinearFactor::shared_ptr makeFactor(gtsam::Key x, gtsam::Key l,
                                                          const Eigen::VectorXd& z,
                                                          const gtsam::SharedNoiseModel& noise) const = 0;

    void setMount(const SensorMount& m) { mount_ = m; }
    const SensorMount& mount() const { return mount_; }

protected:
    SensorMount mount_;
};

}  // namespace fslam
