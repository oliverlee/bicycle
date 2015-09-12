cmake_minimum_required(VERSION 3.0)

project(BIKETEST)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/modules)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3")

add_executable(test_bicycle bicycle.cc test_bicycle.cc)
add_executable(test_kalman bicycle.cc test_kalman.cc)
add_executable(test_lqr bicycle.cc test_lqr.cc)
add_executable(test_lqr_kalman bicycle.cc test_lqr_kalman.cc)
