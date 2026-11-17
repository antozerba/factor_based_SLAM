#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "fslam/core/types.hpp"

namespace fslam {

// Turns one raw sensor frame into a list of measurement vectors z, in the
// layout expected by the sensor's IMeasurementModel.
class IFeatureExtractor {
public:
    virtual ~IFeatureExtractor() = default;

    /// @brief Extractor identifier, e.g. "tree_detector".
    virtual std::string name() const = 0;

    /**
     * @brief Turn one raw sensor frame into measurements  Z_k = { z_k^1, …, z_k^m }.
     *
     * The order of the measurements carries no meaning: which landmark each
     * z_k^j belongs to is decided later by data association.
     *
     * @param frame Raw frame (laser scan, point cloud, ready-made measurements, ...).
     * @return Measurement vectors in the layout of the sensor's IMeasurementModel.
     * @throws std::invalid_argument if the frame carries the wrong payload type.
     */
    virtual std::vector<Eigen::VectorXd> extract(const RawSensorFrame& frame) const = 0;
};

}  // namespace fslam
