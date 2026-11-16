# Examples for timeCont module

# Add the test executables
add_executable(test_factor_lidar_map2pose_interpolation 
    test_factor_lidar_map2pose_interpolation.cpp
)

add_executable(test_factor_graph
    test_factor_graph.cpp
)
add_executable(test_factor_twist_matching
    test_factor_twist_matching.cpp
)

# Link with the timeCont library and dependencies
target_link_libraries(test_factor_lidar_map2pose_interpolation 
    FGraphTime
    FGraph
    SE3
    common
)

target_link_libraries(test_factor_graph
    FGraphTime
    FGraph
    SE3
    common
)

target_link_libraries(test_factor_twist_matching
    FGraphTime
    FGraph
    SE3
    common
)

# Set output directory
set_target_properties(test_factor_lidar_map2pose_interpolation PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set_target_properties(test_factor_graph PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set_target_properties(test_factor_twist_matching PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)