cmake_minimum_required(VERSION 3.10)
project(RealSenseProject)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置第三方库路径
set(THIRDPARTY_DIR "${CMAKE_SOURCE_DIR}/3rdparty")
message(STATUS "THIRDPARTY_DIR: ${THIRDPARTY_DIR}")

# 查找 librealsense2
find_library(REALSENSE2_LIBRARY NAMES realsense2 HINTS "${THIRDPARTY_DIR}/camera_realsense/lib")
find_path(REALSENSE2_INCLUDE_DIR librealsense2/rs.hpp HINTS "${THIRDPARTY_DIR}/camera_realsense/include")
message(STATUS "REALSENSE2_LIBRARY: ${REALSENSE2_LIBRARY}")
message(STATUS "REALSENSE2_INCLUDE_DIR: ${REALSENSE2_INCLUDE_DIR}")
if (NOT REALSENSE2_LIBRARY OR NOT REALSENSE2_INCLUDE_DIR)
    message(FATAL_ERROR "librealsense2 not found. Please install librealsense2 v2.54.2 in ${THIRDPARTY_DIR}/camera_realsense.")
endif()

# 查找 OpenCV
set(OpenCV_DIR "${THIRDPARTY_DIR}/opencv/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_DIR: ${OpenCV_DIR}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

# 查找 Eigen
set(EIGEN3_DIR "${THIRDPARTY_DIR}/eigen/share/eigen3/cmake")
find_path(EIGEN3_INCLUDE_DIR Eigen/Core HINTS "${THIRDPARTY_DIR}/eigen/include/eigen3")
message(STATUS "EIGEN3_DIR: ${EIGEN3_DIR}")
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
if (NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen 3.4.0 not found. Please install Eigen in ${THIRDPARTY_DIR}/eigen.")
endif()

# 设置 PCL 路径
set(PCL_DIR "${THIRDPARTY_DIR}/pcl/share/pcl-1.12")
find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization)## 添加visualization
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

# 设置 VTK 路径（优先尝试 3rdparty/vtk）
set(VTK_DIR "${THIRDPARTY_DIR}/vtk/lib/cmake/vtk-8.2")
find_package(VTK REQUIRED)
message(STATUS "VTK_DIR: ${VTK_DIR}")
message(STATUS "VTK_LIBRARIES: ${VTK_LIBRARIES}")

# 包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${REALSENSE2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${VTK_INCLUDE_DIRS}
)

# 链接目录
link_directories(
    ${PCL_LIBRARY_DIRS}
    ${THIRDPARTY_DIR}/camera_realsense/lib
    ${VTK_LIBRARY_DIRS}
)

# 定义宏
add_definitions(${PCL_DEFINITIONS})


# 添加可执行文件
add_executable(realsenseRUN
    main.cpp
    src/RealSenseCameraInterface.cpp
)

# 链接库
target_link_libraries(realsenseRUN
    ${REALSENSE2_LIBRARY}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    pthread
)

# 设置运行时 RPATH
set_target_properties(realsenseRUN PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "${THIRDPARTY_DIR}/vtk/lib;${THIRDPARTY_DIR}/opencv/lib;${THIRDPARTY_DIR}/pcl/lib;${THIRDPARTY_DIR}/camera_realsense/lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
)