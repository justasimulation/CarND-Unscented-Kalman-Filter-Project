project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources src/main.cpp src/storage.h src/storage.cpp src/measurement_device.h src/app.h src/app.cpp src/fusion_tracker.h src/fusion_tracker.cpp src/kalman_filter_state.h src/laser_filter.h src/laser_filter.cpp src/radar_filter.h src/radar_filter.cpp src/fusion_tracker_result.h src/unscented_kalman_filter.h src/unscented_kalman_filter.cpp)

add_executable(UnscentedKF ${sources})
