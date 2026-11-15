cmake_minimum_required(VERSION 3.20)
project(factor_based_SLAM CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- core: types, math, noise models ------------------------------------------
add_library(fslam_core
        src/core/math.cpp
        src/core/noise_model_factory.cpp
)
target_include_directories(fslam_core PUBLIC include)
target_link_libraries(fslam_core PUBLIC gtsam Eigen3::Eigen yaml-cpp::yaml-cpp)

# --- sensors: motion models, preintegration, measurement models, extractors ----
add_library(fslam_sensors
        src/sensors/preintegrator.cpp
        src/sensors/motion/ackermann_bicycle.cpp
        src/sensors/motion/direct_relative_pose.cpp
        src/sensors/models/range_bearing.cpp
        src/sensors/extractors/pass_through.cpp
        src/sensors/extractors/tree_detector.cpp
)
target_link_libraries(fslam_sensors PUBLIC fslam_core)

# --- tests -----------------------------------------------------------------------
enable_testing()
find_package(GTest REQUIRED)
add_executable(fslam_tests
        tests/test_math.cpp
        tests/test_types.cpp
        tests/test_noise_model_factory.cpp
        tests/test_motion_models.cpp
        tests/test_measurement_models.cpp
        tests/test_feature_extractors.cpp
)
target_link_libraries(fslam_tests PRIVATE fslam_sensors GTest::gtest_main)
target_compile_definitions(fslam_tests PRIVATE FSLAM_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
include(GoogleTest)
gtest_discover_tests(fslam_tests)
