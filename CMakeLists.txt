cmake_minimum_required(VERSION 3.2)

project(RapidCollisionDetection CXX)

set(CMAKE_CXX_STANDARD 11)

# Enable all warnings, treat warnings as errors, compile with highest optimizaiton
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_subdirectory(src)
add_subdirectory(test)
