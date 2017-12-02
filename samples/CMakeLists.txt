cmake_minimum_required(VERSION 3.1)

project(test_p2o)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(Eigen3 REQUIRED)

set (CMAKE_CXX_STANDARD 11)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(
)

add_executable(test_p2o
    ${CMAKE_CURRENT_SOURCE_DIR}/../p2o.h
    test_p2o.cpp
)

target_link_libraries(test_p2o
)

