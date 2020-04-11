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

add_executable(lbvh_simplify_model
  tools/simplify_model.cpp
  third-party/tiny_obj_loader.cc)

target_include_directories(lbvh_simplify_model PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

set(simplified_models
  simplified_model_float.bin
  simplified_model_double.bin)

add_custom_command(OUTPUT ${simplified_models}
  DEPENDS ${model_path} lbvh_simplify_model
  COMMAND $<TARGET_FILE:lbvh_simplify_model> ${model_path}
  COMMENT "Generating simplified models.")

add_custom_target(lbvh_simplified_models ALL
  DEPENDS ${simplified_models})

add_executable(lbvh_test
  lbvh_test.cpp
  lbvh.h
  third-party/stb_image_write.c)

target_compile_options(lbvh_test PRIVATE ${cxxflags})

target_link_libraries(lbvh_test PRIVATE lbvh Threads::Threads)

enable_testing()
