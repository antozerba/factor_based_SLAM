#pragma once

#include "fslam/sensors/ifeature_extractor.hpp"

namespace fslam {

// For datasets that already contain measurements (MeasurementList payload),
// e.g. the GA2 simulated range-bearing data. Returns each z unchanged.
class PassThroughExtractor : public IFeatureExtractor {
public:
    std::string name() const override { return "pass_through"; }
    std::vector<Eigen::VectorXd> extract(const RawSensorFrame& frame) const override;
};

}  // namespace fslam
