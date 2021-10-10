cmake_minimum_required(VERSION 3.0)

project(lbvh)

option(LBVH_EXAMPLES "Whether or not to build the examples." OFF)

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
  third-party/tiny_obj_loader.h
  third-party/tiny_obj_loader.cc)

target_compile_options(lbvh_test PRIVATE ${cxxflags})

target_link_libraries(lbvh_test PRIVATE lbvh Threads::Threads)

if(NOT MSVC)
  target_compile_options(lbvh_test PRIVATE -Wall -Wextra -Werror -Wfatal-errors)
endif(NOT MSVC)

if(LBVH_EXAMPLES)

  add_executable(lbvh_example_minimal
    examples/minimal.cpp)

  target_link_libraries(lbvh_example_minimal PRIVATE lbvh Threads::Threads)

  set_target_properties(lbvh_example_minimal
    PROPERTIES
      OUTPUT_NAME minimal_example)

endif(LBVH_EXAMPLES)

enable_testing()
