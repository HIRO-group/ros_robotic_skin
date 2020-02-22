cmake_minimum_required(VERSION 3.15)
project(PoseEstimation)

set(CMAKE_CXX_STANDARD 14)
add_executable(PoseEstimation main.cpp)
target_include_directories(PoseEstimation PUBLIC /opt/ros/melodic/include)