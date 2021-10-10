#pragma once

#include <memory>

#include <cstddef>

class library
{
public:
  enum class which
  {
    lbvh,
    bvh_sweep_sah,
    bvh_lbvh
  };
  static auto create_library(which w) -> std::unique_ptr<library>;

  virtual ~library() = default;

  virtual void build_bvh(const float* verts, std::size_t tri_count) = 0;

  virtual void render(float* rgb, std::size_t w, std::size_t h) = 0;
};
