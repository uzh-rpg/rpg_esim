cmake_minimum_required(VERSION 2.8.3)
project(imp_planar_renderer)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
catkin_simple()

set(HEADERS
    include/esim/imp_planar_renderer/planar_renderer.hpp
)

set(SOURCES
    src/planar_renderer.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

cs_install()
cs_export()
