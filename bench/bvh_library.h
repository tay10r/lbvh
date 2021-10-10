#pragma once

#include "library.h"

#include <bvh/bvh.hpp>
#include <bvh/triangle.hpp>

class bvh_library final : public library
{
public:
  enum class builder
  {
    lbvh,
    sweep_sah
  };

  bvh_library(builder b)
    : m_builder(b)
  {}

  void build_bvh(const float* verts, std::size_t tri_count) override;

  void render(float* rgb, std::size_t w, std::size_t h) override;

private:
  std::vector<bvh::Triangle<float>> m_triangles;

  bvh::Bvh<float> m_bvh;

  builder m_builder;
};
