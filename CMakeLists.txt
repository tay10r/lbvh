cmake_minimum_required(VERSION 3.0)

project(lbvh)

option(LBVH_EXAMPLES "Whether or not to build the examples." OFF)
option(LBVH_BENCH "Whether or not to build the benchmark." OFF)
option(LBVH_TEST "Whether or not to build the test." OFF)
option(LBVH_DEBUG "Whether or not to build the debug utilities." OFF)

find_package(Threads REQUIRED)

set(model_path "${CMAKE_CURRENT_SOURCE_DIR}/models/sponza.obj")

if(MSVC)
  list(APPEND cxxflags "/DMODEL_PATH=\"${model_path}\"")
else(MSVC)
  list(APPEND cxxflags "-DMODEL_PATH=\"${model_path}\"")
endif(MSVC)

add_library(lbvh INTERFACE)

target_include_directories(lbvh INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

################
# Test Program #
################

if(LBVH_TESTS)

  add_executable(lbvh_test
    lbvh_test.cpp
    lbvh.h
    third-party/stb_image_write.h
    third-party/stb_image_write.c
    third-party/tiny_obj_loader.h
    third-party/tiny_obj_loader.cc)

  target_compile_options(lbvh_test PRIVATE ${cxxflags})

  target_link_libraries(lbvh_test PRIVATE lbvh Threads::Threads)

  if(NOT MSVC)
    target_compile_options(lbvh_test PRIVATE -Wall -Wextra -Werror -Wfatal-errors)
  endif(NOT MSVC)

  enable_testing()

endif(LBVH_TESTS)

if(LBVH_DEBUG)

  add_executable(lbvh_print_graph
    debug/print_graph.cpp
    third-party/tiny_obj_loader.h
    third-party/tiny_obj_loader.cc)

  target_link_libraries(lbvh_print_graph PRIVATE lbvh Threads::Threads)

  set_target_properties(lbvh_print_graph PROPERTIES OUTPUT_NAME print_graph)

endif(LBVH_DEBUG)

if(LBVH_BENCH)

  cmake_minimum_required(VERSION 3.14.7)

  include(FetchContent)

  FetchContent_Declare(bvh URL "https://github.com/madmann91/bvh/archive/master.zip")

  FetchContent_MakeAvailable(bvh)

  find_package(OpenMP REQUIRED COMPONENTS CXX)

  add_executable(lbvh_bench
    bench/main.cpp
    bench/library.h
    bench/library.cpp
    bench/this_library.h
    bench/this_library.cpp
    bench/bvh_library.h
    bench/bvh_library.cpp
    third-party/stb_image_write.h
    third-party/stb_image_write.c
    third-party/tiny_obj_loader.h
    third-party/tiny_obj_loader.cc)

  if(NOT MSVC)
    target_compile_options(lbvh_bench PRIVATE -Wall -Wextra -Werror -Wfatal-errors -Wno-unknown-pragmas)
  endif(NOT MSVC)

  target_compile_features(lbvh_bench PRIVATE cxx_std_17)

  target_link_libraries(lbvh_bench
    PRIVATE
      lbvh
      bvh
      Threads::Threads
      OpenMP::OpenMP_CXX)

endif(LBVH_BENCH)

###################
# Minimal Example #
###################

if(LBVH_EXAMPLES)

  add_executable(lbvh_example_minimal
    examples/minimal.cpp)

  target_link_libraries(lbvh_example_minimal PRIVATE lbvh Threads::Threads)

  set_target_properties(lbvh_example_minimal
    PROPERTIES
      OUTPUT_NAME minimal_example)

endif(LBVH_EXAMPLES)
