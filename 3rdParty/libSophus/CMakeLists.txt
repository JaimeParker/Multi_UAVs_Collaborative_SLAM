cmake_minimum_required(VERSION 3.20)
project(libSophus)

set(CMAKE_CXX_STANDARD 14)

# add Eigen3
include_directories("/usr/local/include/eigen3")

# add Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# add executable
add_executable(libSophus main.cpp useSophus.cpp useSophus.h)

# link libraries
target_link_libraries(libSophus ${Sophus_LIBRARIES} fmt)
