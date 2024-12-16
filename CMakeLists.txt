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

# ros2 node executable
add_executable(${PROJECT_NAME} 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/chad_ros.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/chad_lvr2.cpp")
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/")

# LVR2
include(FetchContent)
FetchContent_Declare(lvr2
    GIT_REPOSITORY "https://gitlab.informatik.uni-osnabrueck.de/Las_Vegas_Reconstruction/Develop.git"
    GIT_TAG "develop"
    GIT_SHALLOW ON)
FetchContent_MakeAvailable(lvr2)
target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/include/")
target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/ext/HighFive/include/")
target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/ext/spdmon/include")
target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/ext/spdlog/include")
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM "${HDF5_CXX_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} lvr2_static ${LVR2_LIB_DEPENDENCIES})

# CHAD TSDF
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/chad_tsdf")
find_package(Eigen3 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} chad::tsdf)
target_link_libraries(${PROJECT_NAME} fmt::fmt)

# package dependencies
find_package(ament_cmake REQUIRED)
list(APPEND ROS_PKGS rclcpp sensor_msgs geometry_msgs pcl_ros)
foreach(PKG ${ROS_PKGS})
    find_package(${PKG} REQUIRED)
endforeach()
# ament package
ament_target_dependencies(${PROJECT_NAME} ${ROS_PKGS})
ament_package()

# install to ros ws lib dir
install(TARGETS ${PROJECT_NAME} DESTINATION "lib/${PROJECT_NAME}")