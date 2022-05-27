cmake_minimum_required(VERSION 3.21)
project(6_Map_Merging)

set(CMAKE_CXX_STANDARD 14)

# Finding Packages
# OpenCV
find_package(OpenCV REQUIRED)
# find catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosbag
        cv_bridge
        )
# Pangolin
find_package(Pangolin)
# Eigen3
find_package(Eigen3)

# Including Directories
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

# Using Project Name as Executable File
add_library(${PROJECT_NAME} SHARED
        main.cpp
        src/Data.cc
        include/Data.h
        src/ORBMatcher.cc
        include/ORBMatcher.h)
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${Eigen3_LIBRARIES})

add_executable(main main.cpp)
target_link_libraries(main ${PROJECT_NAME})


# Additional Parts
# generate ros-bag
add_executable(gen_rosbag src/video2rosbag.cc)
target_link_libraries(gen_rosbag ${OpenCV_LIBS} ${catkin_LIBRARIES})

# SIFT
add_executable(SIFT_extract src/SIFT_extract.cpp)
target_link_libraries(SIFT_extract ${OpenCV_LIBS})

# test KeyBoard staff
add_executable(keyboard src/GetKeyBoard.cc)