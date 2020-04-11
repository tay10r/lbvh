cmake_minimum_required(VERSION 3.0)

project(lbvh)

find_package(Threads REQUIRED)

set(model_path "${CMAKE_CURRENT_SOURCE_DIR}/models/sponza.obj")

if(MSVC)
  list(APPEND cxxflags "/DMODEL_PATH=\"${model_path}\"")
else(MSVC)
  list(APPEND cxxflags "-DMODEL_PATH=\"${model_path}\"")
endif(MSVC)

add_library(lbvh INTERFACE)

target_include_directories(lbvh INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(lbvh_test
  lbvh_test.cpp
  lbvh.h
  third-party/stb_image_write.c
  third-party/tiny_obj_loader.cc)

target_compile_options(lbvh_test PRIVATE ${cxxflags})

target_link_libraries(lbvh_test PRIVATE lbvh Threads::Threads)
