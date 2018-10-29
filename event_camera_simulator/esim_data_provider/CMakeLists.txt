cmake_minimum_required(VERSION 2.8.3)
project(esim_data_provider)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
catkin_simple()

set(HEADERS
    include/esim/data_provider/data_provider_base.hpp
    include/esim/data_provider/data_provider_factory.hpp
    include/esim/data_provider/data_provider_online_render.hpp
    include/esim/data_provider/data_provider_online_simple.hpp
    include/esim/data_provider/data_provider_from_folder.hpp
    include/esim/data_provider/data_provider_rosbag.hpp
    include/esim/data_provider/renderer_factory.hpp
)

set(SOURCES
    src/data_provider_base.cpp
    src/data_provider_factory.cpp
    src/data_provider_online_render.cpp
    src/data_provider_online_simple.cpp
    src/data_provider_from_folder.cpp
    src/data_provider_rosbag.cpp
    src/renderer_factory.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

cs_install()
cs_export()
