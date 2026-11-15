#pragma once

#include <vector>

#include "fslam/sensors/ifeature_extractor.hpp"

namespace fslam {

// Tree detector for 2-D laser scans (Victoria Park). Segments the scan at range
// / angle jumps, removes occluded and too-large segments, and returns one
// [range, bearing] per tree (range to the trunk centre).
// Port of GA2 preprocessing.detect_trees + extract_tree_measurements
// (originally github.com/ramanans1/EKF-SLAM tree_extraction.py).
//
// Angles: beam i is at angle_min + i * angle_increment in the sensor frame.
// GA2 hard-coded 361 beams, 0.5 deg, and subtracted pi/2: that is
// angle_min = -pi/2, angle_increment = pi/360 here.
class TreeDetector : public IFeatureExtractor {
public:
    struct Params {
        double max_range = 50.0;       // drop trees farther than this (GA2 sensor.max_range)
        double max_beam_range = 75.0;  // M11: beams at/above this are "no return"
        double min_range = 1.0;        // M10: ignore objects closer than this
        double edge_margin_rad = 5.0 * M_PI / 306.0;  // daa: ignore segments touching the scan edges
        double range_jump = 1.5;       // M2: range discontinuity that splits segments [m]
        double angle_jump_rad = 10.0 * M_PI / 360.0;  // M2a: angular gap that splits segments
        double merge_distance = 3.0;   // M3: segments this close are probably one object -> drop
        double max_extent = 1.0;       // M5: max distance between segment endpoints [m]
        double min_angle_gap_rad = 2.0 * M_PI / 360.0;  // daMin2 (= 1 deg = 2 beams): occlusion test
    };

    TreeDetector() = default;
    explicit TreeDetector(const Params& p) : p_(p) {}

    std::string name() const override { return "tree_detector"; }
    std::vector<Eigen::VectorXd> extract(const RawSensorFrame& frame) const override;

    // The detector itself, on a bare scan (useful for tests and tools).
    std::vector<Eigen::VectorXd> detect(const LaserScan& scan) const;

    const Params& params() const { return p_; }

private:
    Params p_;
};

}  // namespace fslam
