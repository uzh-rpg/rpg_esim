cmake_minimum_required(VERSION 2.8.3)
project(esim_common)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
catkin_simple()

set(HEADERS
    include/esim/common/types.hpp
    include/esim/common/utils.hpp
)

set(SOURCES
    src/utils.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########

catkin_add_gtest(test_utils test/test_utils.cpp)
target_link_libraries(test_utils ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_install()
cs_export()
