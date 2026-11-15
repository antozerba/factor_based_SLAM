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

    virtual std::string name() const = 0;

    virtual std::vector<Eigen::VectorXd> extract(const RawSensorFrame& frame) const = 0;
};

}  // namespace fslam
