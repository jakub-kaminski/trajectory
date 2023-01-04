add_executable(sample sample.cpp)
target_include_directories(sample PRIVATE
        ${CMAKE_SOURCE_DIR}/include/Trajectory
        ${CMAKE_SOURCE_DIR}/lib/Eigen)

target_link_libraries(sample Trajectory)

add_executable(rotations rotations.cpp)
target_include_directories(rotations PRIVATE
        ${CMAKE_SOURCE_DIR}/include/Trajectory
        ${CMAKE_SOURCE_DIR}/lib/Eigen)

target_link_libraries(rotations Trajectory)
