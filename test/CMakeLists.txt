add_executable(PerformanceEval PerformanceEval.cpp)
target_link_libraries(PerformanceEval RapidCollisionDetection RapidQuadcopterTrajectories Quartic)

add_executable(ForestPerformanceEval ForestPerformanceEval.cpp)
target_link_libraries(ForestPerformanceEval RapidCollisionDetection RapidQuadcopterTrajectories Quartic)

include_directories(${RapidCollisionDetection_SOURCE_DIR}/include)