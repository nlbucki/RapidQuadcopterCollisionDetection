cmake_minimum_required(VERSION 3.2)

project(RapidCollisionDetection CXX)

set(CMAKE_CXX_STANDARD 11)

# Enable all warnings, treat warnings as errors, compile with highest optimizaiton
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic -O3")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(src)
add_subdirectory(test)
