cmake_minimum_required(VERSION 3.20)
project(factor_based_SLAM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(fslam_core src/core/math.cpp)
target_include_directories(fslam_core PUBLIC include)
target_link_libraries(fslam_core PUBLIC gtsam Eigen3::Eigen yaml-cpp)

enable_testing()
find_package(GTest REQUIRED)
add_executable(fslam_tests tests/test_math.cpp)
target_link_libraries(fslam_tests fslam_core GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(fslam_tests)