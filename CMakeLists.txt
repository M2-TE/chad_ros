cmake_minimum_required(VERSION 3.17)
project(chad_ros LANGUAGES CXX)

list(APPEND CMAKE_C_FLAGS "-march=native")
list(APPEND CMAKE_CXX_FLAGS "-march=native")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER_LAUNCHER ccache)
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# package dependencies
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/chad_tsdf")
find_package(ament_cmake REQUIRED)
list(APPEND ROS_PKGS rclcpp sensor_msgs geometry_msgs pcl_ros)
foreach(PKG ${ROS_PKGS})
    find_package(${PKG} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)

# ros2 node executable
add_executable(${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/chad_ros.cpp")

# link dependencies
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} chad::tsdf)
ament_target_dependencies(${PROJECT_NAME} ${ROS_PKGS})
ament_package()

# install to ros ws lib dir
install(TARGETS ${PROJECT_NAME} DESTINATION "lib/${PROJECT_NAME}")