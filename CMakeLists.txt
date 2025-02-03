cmake_minimum_required(VERSION 3.17)
project(chad_ros LANGUAGES C CXX)

list(APPEND CMAKE_C_FLAGS "-march=native")
list(APPEND CMAKE_CXX_FLAGS "-march=native")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
include(FetchContent)

# ros2 node executable
add_executable(${PROJECT_NAME} 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/chad_ros.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/chad_lvr2.cpp")
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/")

# set build options for all TSDF maps
set(VOXEL_RESOLUTION 0.05)
set(BENCHMARKING ON)
set(RECONSTRUCTION OFF)

if    (OFF) # VDBFusion
    # OpenVDB
    set(OPENVDB_BUILD_PYTHON_MODULE OFF) # untested
    set(OPENVDB_BUILD_VDB_PRINT OFF) # untested
    set(OPENVDB_BUILD_AX OFF)
    set(OPENVDB_BUILD_NANOVDB OFF)
    set(OPENVDB_CORE_SHARED ON)
    set(OPENVDB_CORE_STATIC OFF)
    FetchContent_Declare(openvdb
        GIT_REPOSITORY "https://github.com/AcademySoftwareFoundation/openvdb.git"
        GIT_TAG "v11.0.0"
        GIT_SHALLOW ON
        OVERRIDE_FIND_PACKAGE)
    FetchContent_MakeAvailable(openvdb)
    target_link_libraries(${PROJECT_NAME} openvdb_shared)

    # VDBFusion
    FetchContent_Declare(vdbfusion
        GIT_REPOSITORY "https://github.com/PRBonn/vdbfusion.git"
        GIT_TAG "v0.1.6"
        GIT_SHALLOW ON)
    FetchContent_Populate(vdbfusion)
    target_sources(${PROJECT_NAME} PRIVATE 
        "${vdbfusion_SOURCE_DIR}/src/vdbfusion/vdbfusion/VDBVolume.cpp"
        "${vdbfusion_SOURCE_DIR}/src/vdbfusion/vdbfusion/MarchingCubes.cpp")
    target_include_directories(${PROJECT_NAME} PRIVATE "${vdbfusion_SOURCE_DIR}/src/vdbfusion/")

    # IGL for reconstruction
    FetchContent_Declare(igl
        GIT_REPOSITORY "https://github.com/libigl/libigl.git"
        GIT_TAG "v2.5.0"
        GIT_SHALLOW ON)
    FetchContent_MakeAvailable(igl)
    target_link_libraries(${PROJECT_NAME} igl::core)
    set(MAPPING_BACKEND 1)
elseif(OFF) # octomap
    find_package(octomap REQUIRED)
    target_link_libraries(${PROJECT_NAME} octomap octomath)
    set(MAPPING_BACKEND 2)
else() # CHAD TSDF
    set(CUDA_COMPILER /usr/disabled)
    # LVR2
    FetchContent_Declare(lvr2
        GIT_REPOSITORY "https://gitlab.informatik.uni-osnabrueck.de/Las_Vegas_Reconstruction/Develop.git"
        GIT_TAG "develop"
        GIT_SHALLOW ON)
    FetchContent_MakeAvailable(lvr2)
    find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
    target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/include/")
    target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/ext/HighFive/include/")
    target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/ext/spdmon/include")
    target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/ext/spdlog/include")
    target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM "${HDF5_CXX_INCLUDE_DIRS}")
    target_link_libraries(${PROJECT_NAME} lvr2_static ${LVR2_LIB_DEPENDENCIES})

    # CHAD TSDF
    set(CHAD_LEAF_RESOLUTION ${VOXEL_RESOLUTION})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/chad_tsdf")
    find_package(Eigen3 REQUIRED NO_MODULE)
    target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
    target_link_libraries(${PROJECT_NAME} chad::tsdf)
    target_link_libraries(${PROJECT_NAME} fmt::fmt)
    target_link_libraries(${PROJECT_NAME} chad::tsdf)
    set(MAPPING_BACKEND 0)
endif()

# set params as macro defs
target_compile_definitions(${PROJECT_NAME} PUBLIC VOXEL_RESOLUTION=${VOXEL_RESOLUTION})
target_compile_definitions(${PROJECT_NAME} PUBLIC MAPPING_BACKEND=${MAPPING_BACKEND})
if(BENCHMARKING)
    target_compile_definitions(${PROJECT_NAME} PUBLIC BENCHMARKING)
endif()
if(RECONSTRUCTION)
    target_compile_definitions(${PROJECT_NAME} PUBLIC RECONSTRUCTION)
endif()

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