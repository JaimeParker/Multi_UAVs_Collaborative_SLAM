cmake_minimum_required(VERSION 3.21)
project(6_Map_Merging)

set(CMAKE_CXX_STANDARD 14)

# find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosbag
        cv_bridge)
include_directories(${catkin_INCLUDE_DIRS})

# main
add_executable(6_Map_Merging main.cpp)

# generate ros-bag
add_executable(gen_rosbag src/video2rosbag.cc)
target_link_libraries(gen_rosbag ${OpenCV_LIBS} ${catkin_LIBRARIES})
