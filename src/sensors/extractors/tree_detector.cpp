#include "fslam/sensors/extractors/tree_detector.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <variant>

#include "fslam/core/math.hpp"

namespace fslam {

/**
 * @brief Extract one [range, bearing] measurement per tree trunk from a laser frame.
 * @param frame Frame carrying a LaserScan.
 * @return Tree measurements z^j = [r^j, φ^j]ᵀ; see detect().
 * @throws std::invalid_argument if the payload is not a LaserScan.
 */
std::vector<Eigen::VectorXd> TreeDetector::extract(const RawSensorFrame& frame) const {
    const auto* scan = std::get_if<LaserScan>(&frame.payload);
    if (!scan) {
        throw std::invalid_argument("TreeDetector: frame '" + frame.sensor_name +
                                    "' does not carry a LaserScan");
    }
    return detect(*scan);
}

/**
 * @brief Detect tree trunks in a bare 2-D laser scan.
 *
 * Beam i has range r_i and angle a_i = i·Δa from the first beam. The steps are:
 *  -# Keep beams with a return: r_i < max_beam_range.
 *  -# Split into segments where consecutive kept beams jump in range,
 *     |r_{i+1} - r_i| > range_jump, or in angle, a_{i+1} - a_i > angle_jump_rad.
 *  -# Drop segments that are probably parts of a larger object (end of one segment
 *     within merge_distance of the start of one of the next three), and the rear one
 *     of two angularly adjacent segments (partially occluded).
 *  -# Keep segments with endpoints p_s, p_e that are small, ‖p_s - p_e‖ < max_extent,
 *     not too close, away from the scan edges, and roughly round:
 *     @verbatim
 *       d = (a_e - a_s + Δa) · (r_s + r_e) / 2     (arc length ≈ trunk diameter)
 *       |r_s - r_e| < d / 3
 *     @endverbatim
 *  -# For each tree, measure to the trunk centre, not its surface:
 *     @verbatim
 *       r = r_mid + d / 2
 *       φ = ssa( angle_min + (a_s + a_e) / 2 )
 *     @endverbatim
 *     where r_mid is the range of the middle beam(s). Trees with r ≥ max_range
 *     are discarded.
 *
 * Variable names follow GA2 detect_trees so the two can be compared line by line.
 * GA2 works with angles measured from the first beam (0 .. pi); here
 * a_i = i * inc, and angle_min is added back at the very end.
 *
 * @param scan Laser scan (beam i at angle_min + i * angle_increment, sensor frame).
 * @return Tree measurements [r, φ]ᵀ in the sensor frame.
 */
std::vector<Eigen::VectorXd> TreeDetector::detect(const LaserScan& scan) const {
    std::vector<Eigen::VectorXd> trees;
    const std::size_t n = scan.ranges.size();
    if (n < 2) {
        return trees;
    }
    const double inc = scan.angle_increment;

    // Beam angle from the first beam. Mathematically i * inc, but evaluated as
    // GA2 does ((i * pi) / 360 with inc = pi/360): the occlusion test compares
    // 2-beam gaps against exactly 1 deg, so the last bit decides the outcome and
    // i * inc would disagree with GA2 on some scans.
    const double inc360 = inc * 360.0;
    auto beamAngle = [inc360](std::size_t i) { return (static_cast<double>(i) * inc360) / 360.0; };
    const double span = beamAngle(n - 1);  // GA2: pi

    // --- 1. keep beams with a return (RR < M11) -------------------------------
    std::vector<double> R1, A1;  // range and angle of the kept beams
    for (std::size_t i = 0; i < n; ++i) {
        const double r = scan.ranges[i];
        if (r < p_.max_beam_range) {
            R1.push_back(r);
            A1.push_back(beamAngle(i));
        }
    }
    const std::size_t L1 = R1.size();
    if (L1 < 1) {
        return trees;
    }

    // --- 2. split into segments at range or angle jumps ----------------------
    // Segment s spans kept beams [start[s], end[s]] (GA2: ii2, ii2u).
    std::vector<std::size_t> start{0}, end;
    for (std::size_t k = 0; k + 1 < L1; ++k) {
        if (std::abs(R1[k + 1] - R1[k]) > p_.range_jump || (A1[k + 1] - A1[k]) > p_.angle_jump_rad) {
            end.push_back(k);
            start.push_back(k + 1);
        }
    }
    end.push_back(L1 - 1);
    const std::size_t L2 = start.size();

    // Endpoints of every segment (GA2: R2/A2/x2/y2 = start, *u = end).
    std::vector<double> R2(L2), A2(L2), R2u(L2), A2u(L2), x2(L2), y2(L2), x2u(L2), y2u(L2);
    for (std::size_t s = 0; s < L2; ++s) {
        R2[s] = R1[start[s]];
        A2[s] = A1[start[s]];
        R2u[s] = R1[end[s]];
        A2u[s] = A1[end[s]];
        x2[s] = R2[s] * std::cos(A2[s]);
        y2[s] = R2[s] * std::sin(A2[s]);
        x2u[s] = R2u[s] * std::cos(A2u[s]);
        y2u[s] = R2u[s] * std::sin(A2u[s]);
    }

    // --- 3. flag segments to discard ------------------------------------------
    std::vector<bool> flag(L2, false);

    // 3a. Segments 1..3 apart whose end/start are closer than M3 are probably
    //     parts of one larger object (wall, bush): drop both.
    const double M3c = p_.merge_distance * p_.merge_distance;
    for (std::size_t lag = 1; lag <= 3; ++lag) {
        if (L2 <= lag) break;
        for (std::size_t i = 0; i + lag < L2; ++i) {
            const double dx = x2[i + lag] - x2u[i];
            const double dy = y2[i + lag] - y2u[i];
            if (dx * dx + dy * dy < M3c) {
                flag[i] = true;
                flag[i + lag] = true;
            }
        }
    }

    // 3b. Neighbouring segments that are adjacent in angle: the one behind is
    //     partially occluded, so its extent is unreliable -> drop it.
    if (L2 > 1) {
        for (std::size_t i = 0; i + 1 < L2; ++i) {
            if ((A2[i + 1] - A2u[i]) < p_.min_angle_gap_rad) {
                const bool next_is_behind = R2[i + 1] > R2u[i];
                flag[i + (next_is_behind ? 1 : 0)] = true;
            }
        }
    }

    // --- 4. keep small, unflagged segments away from the scan edges -----------
    const double M5c = p_.max_extent * p_.max_extent;
    for (std::size_t s = 0; s < L2; ++s) {
        if (flag[s]) continue;

        // 4a. endpoints closer than M5 (a trunk, not a wall)
        const double dx = x2[s] - x2u[s];
        const double dy = y2[s] - y2u[s];
        if (!(dx * dx + dy * dy < M5c)) continue;

        // 4b. not too close, not cut by the edges of the field of view
        if (!(R2[s] > p_.min_range && A2[s] > p_.edge_margin_rad && A2u[s] < (span - p_.edge_margin_rad))) {
            continue;
        }

        // 4c. roughly round: range difference across the segment small vs its width
        const double dL5 = (A2u[s] + inc - A2[s]) * (R2[s] + R2u[s]) / 2.0;  // arc length ~ diameter
        if (!(std::abs(R2[s] - R2u[s]) < dL5 / 3.0)) continue;

        // --- 5. measurement: range to the trunk centre, bearing to the middle ---
        const double mid = (static_cast<double>(start[s]) + static_cast<double>(end[s])) / 2.0;
        const auto iia = static_cast<std::size_t>(std::floor(mid));
        const auto iib = static_cast<std::size_t>(std::ceil(mid));
        const double Rs = (R1[iia] + R1[iib]) / 2.0;

        const double range = Rs + dL5 / 2.0;
        const double bearing = math::wrapAngle(scan.angle_min + (A2[s] + A2u[s]) / 2.0);

        if (range < p_.max_range) {  // GA2 extract_tree_measurements filter
            trees.push_back(Eigen::Vector2d(range, bearing));
        }
    }
    return trees;
}

}  // namespace fslam
