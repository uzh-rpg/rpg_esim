cmake_minimum_required(VERSION 2.8.3)
project(esim_ros)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple()

set(HEADERS
)

set(SOURCES
    src/esim_node.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# make the executable
cs_add_executable(esim_node  src/esim_node.cpp)

# link the executable to the necessary libs
target_link_libraries(esim_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

cs_install()
