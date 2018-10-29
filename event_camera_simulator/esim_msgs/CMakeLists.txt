cmake_minimum_required(VERSION 2.8.3)
project("esim_msgs" CXX)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_install()
cs_export()
