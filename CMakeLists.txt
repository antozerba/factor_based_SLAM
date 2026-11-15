cmake_minimum_required(VERSION 3.20)
project(factor_based_SLAM CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(fslam_core
        src/core/math.cpp
        src/core/noise_model_factory.cpp
)
target_include_directories(fslam_core PUBLIC include)
target_link_libraries(fslam_core PUBLIC gtsam Eigen3::Eigen yaml-cpp::yaml-cpp)

enable_testing()
find_package(GTest REQUIRED)
add_executable(fslam_tests
        tests/test_math.cpp
        tests/test_types.cpp
        tests/test_noise_model_factory.cpp
)
target_link_libraries(fslam_tests PRIVATE fslam_core GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(fslam_tests)
