cmake_minimum_required(VERSION 2.8.3)
project(esim_rendering)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
catkin_simple()

set(HEADERS
    include/esim/rendering/renderer_base.hpp
    include/esim/rendering/simple_renderer_base.hpp
)

set(SOURCES
    src/renderer_base.cpp
    src/simple_renderer_base.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

cs_install()
cs_export()
