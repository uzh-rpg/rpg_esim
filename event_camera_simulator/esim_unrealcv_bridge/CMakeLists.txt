cmake_minimum_required(VERSION 2.8.3)
project(esim_unrealcv_bridge)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
catkin_simple()

add_definitions(-std=c++11)

set(HEADERS
    include/esim/unrealcv_bridge/unrealcv_bridge.hpp
)

set(SOURCES
    src/unrealcv_bridge.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_DL_LIBS} -lpthread -lboost_filesystem -lboost_system)

##########
# TESTS #
##########

cs_add_executable(test_unrealcv_bridge test/test_unrealcv_bridge.cpp)
target_link_libraries(test_unrealcv_bridge ${PROJECT_NAME})


cs_install()
cs_export()
