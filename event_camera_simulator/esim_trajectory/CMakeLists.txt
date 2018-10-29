cmake_minimum_required(VERSION 2.8.3)
project(esim_trajectory)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(HEADERS
    include/esim/trajectory/trajectory_factory.hpp
    include/esim/trajectory/imu_factory.hpp
)

set(SOURCES
    src/trajectory_factory.cpp
    src/imu_factory.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

cs_install()
cs_export()
