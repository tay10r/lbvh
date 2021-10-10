#include <lbvh.h>

#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.1415f
#endif

namespace {

//! In order to build a BVH, we'll
//! need to define a shape to make a scene with.
//! In this example, we choose a sphere because it's simple.
struct sphere final
{
  //! The radius of the sphere.
  float radius;
  //! The position of the sphere,
  //! in X, Y, Z format.
  float pos[3];
};

struct sphere_hit final
{
  float tmin = std::numeric_limits<float>::infinity();

  float uv[2]{ 0, 0 };

  float normal[3]{ 0, 0, 0 };

  constexpr bool operator<(const sphere_hit& other) const noexcept
  {
    return tmin < other.tmin;
  }

  constexpr bool operator<(float distance) const noexcept
  {
    return tmin < distance;
  }
};

} // namespace

int
main()
{
  // This is the scene we want to build
  // a BVH for, just three spheres.
  sphere spheres[3]{ { 1, { 5, 1, -5 } },
                     { 1, { 0, 2, -5 } },
                     { 1, { -5, 1, -5 } } };

  // This is the lamba function we'll be
  // using to get the bounding boxes of our spheres.
  // The library needs bounding boxes in order to construct the BVH.
  //
  // Notice the trailing return type. The term "aabb" is an
  // acronym for axis-aligned bounding box.
  auto sphere_to_box = [](sphere s) -> lbvh::aabb<float> {
    return lbvh::aabb<float>{
      { s.pos[0] - s.radius, s.pos[1] - s.radius, s.pos[2] - s.radius },
      { s.pos[0] + s.radius, s.pos[1] + s.radius, s.pos[2] + s.radius }
    };
  };

  // A builder is what's used to make a BVH from a scene.
  // It can be thought of as a "function object", because its
  // only method is the function call operator.
  lbvh::builder<float> builder;

  // Here we call the builder to make a BVH.
  // We pass it our array of spheres as well
  // as there sphere to box converter and our BVH is made.
  auto bvh = builder(spheres, 3, sphere_to_box);

  // At this point, the main purpose of this library
  // has been accomplished - the BVH has been built.
  // You can traverse the BVH with your own classes.
  //
  // You can also use the built in traverser class, so
  // that you don't have to learn too much about the BVH
  // structure to use it. The rest of this code this completely
  // optional.

  lbvh::traverser<float, sphere, sphere_hit> traverser(bvh, spheres);

  // To use the traverser, we need to define a function
  // that checks for intersection between a ray and our
  // geometric primitive, which is a sphere in this example.

  auto intersect_sphere = [](const sphere& s,
                             const lbvh::ray<float>& ray) -> sphere_hit {
    // We'll piggy back off the math code
    // that's already in lbvh.h, to avoid
    // writing some extra code.

    using namespace lbvh::math;

    lbvh::vec3<float> s_pos{ s.pos[0], s.pos[1], s.pos[2] };

    auto dist = ray.pos - s_pos;

    auto a = dot(ray.dir, ray.dir);

    auto b = 2 * dot(ray.dir, dist);

    auto c = dot(dist, dist) - (s.radius * s.radius);

    // solve quadratic equation

    auto disc = (b * b) - (4 * a * c);
    if (disc < 0)
      return sphere_hit{};

    auto t0 = (-b + std::sqrt(disc)) / (2 * a);
    auto t1 = (-b - std::sqrt(disc)) / (2 * a);

    auto t = t0;

    if (t < 0) {
      t = t1;
    }

    auto hit_pos = ray.pos + (ray.dir * t);

    auto normal = normalize(hit_pos - s_pos);

    constexpr auto pi = float(M_PI);

    lbvh::vec2<float> uv{ (1.0f + std::atan2(normal.z, normal.x) / pi) * 0.5f,
                          std::acos(normal.y) / pi };

    return sphere_hit{ t, { uv.x, uv.y }, { normal.x, normal.y, normal.z } };
  };

  constexpr lbvh::size_type w = 1920;
  constexpr lbvh::size_type h = 1080;

  using channel_type = unsigned char;

  std::vector<channel_type> image_buf(w * h * 3);

  auto aspect_ratio = float(w) / h;

  auto fov = 0.75f;

  for (lbvh::size_type y = 0; y < h; y++) {
    for (lbvh::size_type x = 0; x < w; x++) {
      auto x_ndc = (2 * (x + 0.5f) / float(w)) - 1;
      auto y_ndc = -(2 * (y + 0.5f) / float(h)) + 1;

      lbvh::vec3<float> ray_dir{ x_ndc * aspect_ratio * fov, y_ndc * fov, -1 };

      lbvh::ray<float> r{ { 0, 0, 5 }, ray_dir };

      auto isect = traverser(r, intersect_sphere);
      if (isect) {
        auto offset = ((y * w) + x) * 3;
        image_buf[offset + 0] = channel_type(isect.info.uv[0] * 255);
        image_buf[offset + 1] = channel_type(isect.info.uv[1] * 255);
        image_buf[offset + 2] =
          channel_type(1 - isect.info.uv[0] - isect.info.uv[1]) * 255;
      }
    }
  }

  // We'll write the image to a file so
  // that we can see the result.

  FILE* file = std::fopen("result.ppm", "wr");

  std::fprintf(file, "P6\n%lu\n%lu\n255\n", w, h);

  std::fwrite(image_buf.data(), 1, image_buf.size(), file);

  std::fclose(file);

  return 0;
}
