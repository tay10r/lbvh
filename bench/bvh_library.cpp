#include "bvh_library.h"

#include <bvh/linear_bvh_builder.hpp>
#include <bvh/primitive_intersectors.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/sweep_sah_builder.hpp>

void
bvh_library::build_bvh(const float* vertices, std::size_t tri_count)
{
  using vec3f = bvh::Vector3<float>;

  m_triangles.resize(tri_count);

  for (size_t i = 0; i < tri_count; i++) {
    vec3f p0{ vertices[(i * 9) + 0],
              vertices[(i * 9) + 1],
              vertices[(i * 9) + 2] };

    vec3f p1{ vertices[(i * 9) + 3],
              vertices[(i * 9) + 4],
              vertices[(i * 9) + 5] };

    vec3f p2{ vertices[(i * 9) + 6],
              vertices[(i * 9) + 7],
              vertices[(i * 9) + 8] };

    m_triangles[i] = bvh::Triangle<float>(p0, p1, p2);
  }

  auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(
    m_triangles.data(), m_triangles.size());

  auto global_bbox =
    bvh::compute_bounding_boxes_union(bboxes.get(), m_triangles.size());

  if (m_builder == builder::sweep_sah) {
    bvh::SweepSahBuilder<bvh::Bvh<float>> builder(m_bvh);
    builder.build(global_bbox, bboxes.get(), centers.get(), m_triangles.size());
  } else if (m_builder == builder::lbvh) {
    bvh::LinearBvhBuilder<bvh::Bvh<float>, uint32_t> builder(m_bvh);
    builder.build(global_bbox, bboxes.get(), centers.get(), m_triangles.size());
  }
}

void
bvh_library::render(float* rgb, std::size_t w, std::size_t h)
{
  using vec3f = bvh::Vector3<float>;

  using intersector_type =
    bvh::ClosestPrimitiveIntersector<bvh::Bvh<float>, bvh::Triangle<float>>;

  const int max = w * h;

  const float u_scale = 1.0f / w;
  const float v_scale = 1.0f / h;

  const vec3f ray_origin{ 0, 0, 5 };

  const float aspect = float(w) / h;

#pragma omp parallel for

  for (int i = 0; i < max; i++) {

    const int x = i % w;
    const int y = i / w;

    const float u = (x + 0.5f) * u_scale;
    const float v = (y + 0.5f) * v_scale;

    const float dx = ((u * 2) - 1) * aspect;
    const float dy = (1 - (v * 2));
    const float dz = -1;

    vec3f ray_direction = bvh::normalize(vec3f(dx, dy, dz));

    bvh::Ray<float> ray(ray_origin, ray_direction);

    intersector_type intersector(m_bvh, m_triangles.data());

    bvh::SingleRayTraverser<bvh::Bvh<float>> traverser(m_bvh);

    if (auto hit = traverser.traverse(ray, intersector)) {
      rgb[(i * 3) + 0] = hit->intersection.u;
      rgb[(i * 3) + 1] = hit->intersection.v;
      rgb[(i * 3) + 2] = 1;
    }
  }
}
