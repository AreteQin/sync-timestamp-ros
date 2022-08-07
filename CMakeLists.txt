cmake_minimum_required(VERSION 3.9)
project(sync-timestamp-ros)# LANGUAGES CUDA CXX)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++17")# -O2 -pthread") # -O3: usage is unknown
# set(CMAKE_CXX_FLAGS  "-fopenmp") unknown usage
set(CMAKE_BUILD_TYPE "DEBUG")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        tf2
        tf2_ros
        tf
        cv_bridge
        message_filters
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(publisher src/publisher.cpp)
#target_compile_options(publisher PRIVATE -fPIC)
target_link_libraries(publisher
        ${catkin_LIBRARIES}
        Sophus::Sophus
        )

add_executable(receiver src/receiver.cpp)
target_link_libraries(receiver
        ${catkin_LIBRARIES}
        Sophus::Sophus
        )

add_executable(pure_image_receiver src/pure_image_receiver.cpp)
target_link_libraries(pure_image_receiver
        ${catkin_LIBRARIES}
        )

add_executable(pure_pose_receiver src/pure_pose_receiver.cpp)
target_link_libraries(pure_pose_receiver
        ${catkin_LIBRARIES}
        Sophus::Sophus
        )