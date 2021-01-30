cmake_minimum_required(VERSION 3.13)
project(inverse_pose)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


set(SOURCE_FILES InversePose.cpp)
add_executable(inverse_pose ${SOURCE_FILES})