cmake_minimum_required(VERSION 2.8.3)
project(imp_multi_objects_2d)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
catkin_simple()

set(HEADERS
    include/esim/imp_multi_objects_2d/imp_multi_objects_2d_renderer.hpp
    include/esim/imp_multi_objects_2d/object.hpp
)

set(SOURCES
    src/imp_multi_objects_2d_renderer.cpp
    src/object.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

cs_install()
cs_export()
