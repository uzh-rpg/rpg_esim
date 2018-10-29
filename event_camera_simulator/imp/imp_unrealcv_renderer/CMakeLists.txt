cmake_minimum_required(VERSION 2.8.3)
project(imp_unrealcv_renderer)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
catkin_simple()

set(HEADERS
    include/esim/imp_unrealcv_renderer/utils.hpp
    include/esim/imp_unrealcv_renderer/unrealcv_renderer.hpp
)

set(SOURCES
    src/unrealcv_renderer.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_DL_LIBS} -lpthread -lboost_filesystem -lboost_system)

cs_install()
cs_export()
