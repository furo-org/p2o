cmake_minimum_required(VERSION 3.10)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

project(p2o)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

include_directories(include
        ${EIGEN3_INCLUDE_DIR}
)

add_library(p2o SHARED src/p2o.cpp include/p2o.h)
add_executable(sample_run_p2o samples/sample_run_p2o.cpp)
target_link_libraries(sample_run_p2o p2o Eigen3::Eigen)

find_package(PCL 1.14)
set(BUILD_VIEWER OFF CACHE BOOL "Build p2o viewer")
if (PCL_FOUND)
    set(BUILD_VIEWER ON)
endif(PCL_FOUND)

if(BUILD_VIEWER)
    add_subdirectory(p2o_viewer)
endif(BUILD_VIEWER)

