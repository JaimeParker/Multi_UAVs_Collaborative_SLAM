cmake_minimum_required(VERSION 3.20)
project(libEigen)

set(CMAKE_CXX_STANDARD 14)

add_executable(libEigen main.cpp)


# add headers
include_directories("/usr/include/eigen3")
include_directories("/usr/local/include/eigen3")