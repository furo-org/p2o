cmake_minimum_required(VERSION 3.10)
project(p2o_viewer)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(PCL 1.14 REQUIRED COMPONENTS visualization common io filters registration)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(p2o_viewer
        main.cpp
        mainwindow.h
        mainwindow.cpp
        mainwindow.ui
        load_p2o_file.h
        load_p2o_file.cpp
        ${MOC_SOURCES}
        ${UI_HEADERS}
        ../src/p2o.cpp
)

target_link_libraries(p2o_viewer
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    ${PCL_LIBRARIES}
)

set_property(TARGET p2o_viewer PROPERTY AUTOMOC ON)
set_property(TARGET p2o_viewer PROPERTY AUTOUIC ON)

