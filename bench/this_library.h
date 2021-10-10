#pragma once

#include "library.h"

#include <lbvh.h>

class this_library final : public library
{
public:
  void build_bvh(const float* verts, std::size_t tri_count) override;

  void render(float* rgb, std::size_t w, std::size_t h) override;

private:
  std::vector<lbvh::triangle<float>> m_triangles;

  std::unique_ptr<lbvh::bvh<float>> m_bvh;
};
