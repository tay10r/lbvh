#include "this_library.h"

void
this_library::build_bvh(const float* vertices, std::size_t tri_count)
{
  m_triangles.resize(tri_count);

  for (size_t i = 0; i < tri_count; i++) {
    lbvh::vec3<float> p0{ vertices[(i * 9) + 0],
                          vertices[(i * 9) + 1],
                          vertices[(i * 9) + 2] };

    lbvh::vec3<float> p1{ vertices[(i * 9) + 3],
                          vertices[(i * 9) + 4],
                          vertices[(i * 9) + 5] };

    lbvh::vec3<float> p2{ vertices[(i * 9) + 6],
                          vertices[(i * 9) + 7],
                          vertices[(i * 9) + 8] };

    m_triangles[i] = lbvh::triangle<float>(p0, p1, p2);
  }

  lbvh::triangle_aabb_converter<float> aabb_converter;

  lbvh::builder<float> builder;

  auto bvh = builder(m_triangles.data(), m_triangles.size(), aabb_converter);

  m_bvh.reset(new lbvh::bvh<float>(std::move(bvh)));
}

void
this_library::render(float* rgb, std::size_t w, std::size_t h)
{
  const int max = w * h;

  const float u_scale = 1.0f / w;
  const float v_scale = 1.0f / h;

  const lbvh::vec3<float> ray_origin{ 0, 0, 5 };

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

    lbvh::vec3<float> ray_direction =
      lbvh::math::normalize(lbvh::vec3<float>{ dx, dy, dz });

    lbvh::ray<float> ray{ ray_origin, ray_direction };

    lbvh::traverser<float,
                    lbvh::triangle<float>,
                    lbvh::triangle_intersection<float>>
      traverser(*m_bvh, m_triangles.data());

    lbvh::triangle_intersector<float> intersector;

    auto isect = traverser(ray, intersector);
    if (isect) {
      rgb[(i * 3) + 0] = isect.info.uv.x;
      rgb[(i * 3) + 1] = isect.info.uv.y;
      rgb[(i * 3) + 2] = 1;
    }
  }
}
