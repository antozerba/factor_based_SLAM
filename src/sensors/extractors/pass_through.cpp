#include "fslam/sensors/extractors/pass_through.hpp"

#include <stdexcept>
#include <variant>

namespace fslam {

/**
 * @brief Return the dataset's ready-made measurements unchanged.
 *
 * Z_k = { z_k^1, …, z_k^m } is copied straight from the
 * MeasurementList payload; any labelled landmark ids are dropped, so data association
 * still has to be done.
 *
 * @param frame Frame carrying a MeasurementList.
 * @return The z_k^j, in dataset order.
 * @throws std::invalid_argument if the payload is not a MeasurementList.
 */
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
