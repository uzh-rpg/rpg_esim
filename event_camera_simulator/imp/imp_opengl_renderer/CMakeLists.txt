cmake_minimum_required(VERSION 3.0)
project(imp_opengl_renderer)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glfw3 REQUIRED)
find_package(assimp REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
catkin_simple()

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libraries)

configure_file(configuration/root_directory.h.in configuration/root_directory.h)
include_directories(${CMAKE_BINARY_DIR}/configuration)

# glad
set(GLAD_DIR ${LIB_DIR}/glad)
add_library(glad ${GLAD_DIR}/src/glad.c)
target_include_directories(glad PRIVATE ${GLAD_DIR}/include)


set(SOURCES
    src/opengl_renderer.cpp)

set(SHADERS
    src/shader.vert
    src/shader.frag
    src/optic_flow_shader.frag)

set(HEADERS
    include/esim/imp_opengl_renderer/opengl_renderer.hpp
   )

# make the executable
cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${SHADERS})

# link the executable to the necessary libs
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${GLAD_DIR}/include)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_DL_LIBS} glfw glad assimp)

cs_install()
cs_export()
