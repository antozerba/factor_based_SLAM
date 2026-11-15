#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

namespace fslam {

// Time in seconds. Readers convert from the dataset's unit (e.g. ms) to this.
using Timestamp = double;

// ============================================================================
// Raw stage: produced by IDatasetReader, no processing applied
// ============================================================================

// One odometry reading covering the interval [t0, t1].
// The meaning of u depends on the IMotionModel:
//   ackermann_bicycle     -> [v, steering]
//   direct_relative_pose  -> [dx, dy, dtheta]
//   differential_drive    -> [v_left, v_right]
struct MotionSample {
    Timestamp t0 = 0.0;
    Timestamp t1 = 0.0;
    Eigen::VectorXd u;

    double dt() const { return t1 - t0; }
};

// One landmark observation. Defined here because MeasurementList (a raw
// payload) contains it; it is also the processed-stage measurement.
struct Measurement {
    Eigen::VectorXd z;                       // size = IMeasurementModel::dim()
    int sensor_id = -1;                      // index into SensorSuite
    Timestamp t = 0.0;
    std::optional<int> known_landmark_id;    // only for datasets with labelled landmarks
};

// Raw 2-D lidar sweep. Beam i points at angle_min + i * angle_increment.
struct LaserScan {
    std::vector<float> ranges;
    float angle_min = 0.0f;
    float angle_increment = 0.0f;
    float range_max = 0.0f;
};

// Raw 2-D points in the sensor frame.
struct PointCloud2D {
    std::vector<gtsam::Point2> points;
};

// Measurements that the dataset already provides (e.g. simulated range-bearing).
struct MeasurementList {
    std::vector<Measurement> items;
};

// One reading from one landmark sensor, before feature extraction.
// To support a new kind of raw data, add an alternative to the variant.
struct RawSensorFrame {
    std::string sensor_name;                 // matches a "sensors:" entry in the YAML
    Timestamp t = 0.0;
    std::variant<LaserScan, PointCloud2D, MeasurementList> payload;
};

// Everything IDatasetReader::next() returns for one step.
struct RawStep {
    std::size_t index = 0;
    Timestamp t = 0.0;
    std::vector<MotionSample> motion;        // all odometry since the previous step
    std::vector<RawSensorFrame> frames;      // zero or more sensor readings
};

// ============================================================================
// Processed stage: produced by StepAssembler, consumed by SlamSystem
// ============================================================================

// Preintegrated odometry between two consecutive poses -> one BetweenFactor<Pose2>.
struct RelativeMotion {
    gtsam::Pose2 delta;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    Timestamp t0 = 0.0;
    Timestamp t1 = 0.0;
};

// All measurements from one sensor at one step (one local map / JCBB run each).
struct SensorFrame {
    int sensor_id = -1;
    Timestamp t = 0.0;
    std::vector<Measurement> measurements;
};

// The only input of SlamSystem::processStep.
struct StepInput {
    std::size_t step = 0;
    Timestamp t = 0.0;
    std::optional<RelativeMotion> motion;    // empty at the first step
    std::vector<SensorFrame> frames;
};

// Initial pose and its uncertainty -> PriorFactor<Pose2> on X(0).
struct PosePrior {
    gtsam::Pose2 mean;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
};

}  // namespace fslam
