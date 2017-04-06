cmake_minimum_required(VERSION 2.8.3)
project(aruco_extrinsic)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
        roscpp
        image_transport
        cv_bridge
        )
include_directories(include/ ${catkin_INCLUDE_DIRS})

find_package(OpenCV 3.1.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
#   LIBRARIES aruco_extrinsic
#   CATKIN_DEPENDS other_catkin_pkg
#   DEPENDS system_lib
)

add_executable(aruco_extrinsic
        src/aruco_extrinsic.cpp
        src/ArucoBoardDetector.cpp
        )
target_link_libraries(aruco_extrinsic
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

# Dummy target for CLion
FILE (GLOB_RECURSE clion_all_headers
        ${CMAKE_SOURCE_DIR}/**/*.hpp
        ${CMAKE_SOURCE_DIR}/**/*.h)
ADD_CUSTOM_TARGET(all_clion SOURCES ${clion_all_headers})
