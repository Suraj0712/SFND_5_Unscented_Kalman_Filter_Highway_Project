#minimum version
cmake_minimum_required(VERSION 2.8.3)

#project name (can be anything)
project(Eigen_2d_kalman_filter)

#forgot to include to use c++ 11
add_compile_options(-std=c++11)

#this is telling it to look in /include for your header files
include_directories(include)

#this is telling it to compile src/main and src/Sally as part of your project "sally"
add_executable(${PROJECT_NAME} src/kalman_filter.cpp  src/kalman_filter.h  src/main.cpp  src/measurement_package.h  src/tracking.cpp  src/tracking.h)
