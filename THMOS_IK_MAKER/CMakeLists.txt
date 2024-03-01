cmake_minimum_required(VERSION 3.0)
project(thmos_leg_ik)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)

# Find dependencies
find_package(Eigen3 REQUIRED)

# Include directories
include_directories("/usr/include/eigen3") #(${Eigen3_INCLUDE_DIRS})

# Add library
add_library(thmos_leg_ik SHARED thmos_leg_ik_raw.cpp)
target_link_libraries(thmos_leg_ik ${Eigen3_LIBRARIES})
