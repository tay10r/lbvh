cmake_minimum_required(VERSION 3.0)

project(lbvh CXX)

add_library(lbvh INTERFACE)

target_include_directories(lbvh INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(lbvh_test lbvh_test.cpp lbvh.h models/tiny_obj_loader.cc)

target_link_libraries(lbvh_test PRIVATE lbvh)
