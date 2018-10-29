cmake_minimum_required(VERSION 2.8.3)
project(esim)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple()

set(HEADERS
    include/esim/esim/simulator.hpp
    include/esim/esim/event_simulator.hpp
    include/esim/esim/camera_simulator.hpp
)

set(SOURCES
    src/simulator.cpp
    src/event_simulator.cpp
    src/camera_simulator.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########

catkin_add_gtest(test_event_simulator test/test_event_simulator.cpp)
target_link_libraries(test_event_simulator ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_install()
cs_export()
