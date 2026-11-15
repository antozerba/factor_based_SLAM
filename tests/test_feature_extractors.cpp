// TreeDetector (regression against GA2 detect_trees on real Victoria Park scans)
// and PassThroughExtractor.
// Fixture: tests/data/tree_detector_vp.txt, made by tools/dump_tree_detector_fixture.py.

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "fslam/sensors/extractors/pass_through.hpp"
#include "fslam/sensors/extractors/tree_detector.hpp"

using fslam::LaserScan;
using fslam::RawSensorFrame;
using fslam::TreeDetector;

namespace {

struct ScanCase {
    int index = 0;
    LaserScan scan;
    std::vector<Eigen::Vector2d> trees;  // GA2 output
};

std::vector<ScanCase> loadFixture(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open fixture " + path);

    std::vector<ScanCase> cases;
    std::string tag;
    while (in >> tag) {
        ScanCase c;
        std::size_t beams = 0, m = 0;
        if (tag != "SCAN") throw std::runtime_error("expected SCAN, got " + tag);
        in >> c.index >> beams;
        c.scan.ranges.resize(beams);
        for (float& r : c.scan.ranges) in >> r;
        c.scan.angle_min = -M_PI / 2;
        c.scan.angle_increment = M_PI / 360;
        c.scan.range_max = 80.0;

        in >> tag >> m;
        if (tag != "TREES") throw std::runtime_error("expected TREES, got " + tag);
        c.trees.resize(m);
        for (auto& t : c.trees) in >> t(0) >> t(1);
        cases.push_back(std::move(c));
    }
    return cases;
}

}  // namespace

TEST(TreeDetector, MatchesGa2OnVictoriaParkScans) {
    const auto cases = loadFixture(std::string(FSLAM_TEST_DATA_DIR) + "/tree_detector_vp.txt");
    ASSERT_FALSE(cases.empty());

    const TreeDetector detector;  // defaults = GA2 constants, max_range 50
    std::size_t total = 0;
    for (const ScanCase& c : cases) {
        const auto z = detector.detect(c.scan);
        ASSERT_EQ(z.size(), c.trees.size()) << "scan " << c.index;
        for (std::size_t i = 0; i < z.size(); ++i) {
            // Same float32 ranges as GA2; only summation order differs.
            EXPECT_NEAR(z[i](0), c.trees[i](0), 1e-9) << "scan " << c.index << " tree " << i << " range";
            EXPECT_NEAR(z[i](1), c.trees[i](1), 1e-12) << "scan " << c.index << " tree " << i << " bearing";
        }
        total += z.size();
    }
    EXPECT_GT(total, 0u);
}

TEST(TreeDetector, EmptyAndAllMaxRangeScansGiveNothing) {
    const TreeDetector detector;
    LaserScan scan;
    scan.angle_min = -M_PI / 2;
    scan.angle_increment = M_PI / 360;
    EXPECT_TRUE(detector.detect(scan).empty());

    scan.ranges.assign(361, 80.0f);  // no returns
    EXPECT_TRUE(detector.detect(scan).empty());
}

TEST(TreeDetector, ExtractRequiresLaserScan) {
    RawSensorFrame f;
    f.payload = fslam::MeasurementList{};
    EXPECT_THROW(TreeDetector().extract(f), std::invalid_argument);
}

TEST(PassThroughExtractor, ReturnsMeasurementsUnchanged) {
    fslam::MeasurementList list;
    list.items.push_back({Eigen::Vector2d(3.0, 0.1), 0, 0.0, std::nullopt});
    list.items.push_back({Eigen::Vector2d(8.0, -0.4), 0, 0.0, std::nullopt});
    RawSensorFrame f;
    f.payload = list;

    const auto z = fslam::PassThroughExtractor().extract(f);
    ASSERT_EQ(z.size(), 2u);
    EXPECT_TRUE(z[1].isApprox(Eigen::Vector2d(8.0, -0.4)));
}

TEST(PassThroughExtractor, RejectsRawScans) {
    RawSensorFrame f;
    f.payload = LaserScan{};
    EXPECT_THROW(fslam::PassThroughExtractor().extract(f), std::invalid_argument);
}
