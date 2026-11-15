#include "fslam/sensors/extractors/pass_through.hpp"

#include <stdexcept>
#include <variant>

namespace fslam {

std::vector<Eigen::VectorXd> PassThroughExtractor::extract(const RawSensorFrame& frame) const {
    const auto* list = std::get_if<MeasurementList>(&frame.payload);
    if (!list) {
        throw std::invalid_argument("PassThroughExtractor: frame '" + frame.sensor_name +
                                    "' does not carry a MeasurementList");
    }
    std::vector<Eigen::VectorXd> out;
    out.reserve(list->items.size());
    for (const Measurement& m : list->items) {
        out.push_back(m.z);
    }
    return out;
}

}  // namespace fslam
