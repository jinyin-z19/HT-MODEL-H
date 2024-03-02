cmake_minimum_required(VERSION 3.0)
project(model_h_leg_ik)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)

# Find dependencies
find_package(Eigen3 REQUIRED)

# Include directories
include_directories("/usr/include/eigen3") # for ubuntu

# Add library
add_library(model_h_leg_ik SHARED model_h_leg_ik.cpp)
target_link_libraries(model_h_leg_ik ${Eigen3_LIBRARIES})
