#pragma once

#include <lbvh.h>

#include "third-party/tiny_obj_loader.h"

namespace lbvh {

//! Represents a 3D triangle from an .obj model.
//!
//! \tparam scalar_type The floating point type to represent the triangle with.
template <typename scalar_type>
struct triangle final {
  //! The triangle position values.
  vec3<scalar_type> pos[3];
  //! The UV coordinates at each position.
  vec2<scalar_type> uv[3];
};

//! Represents a 3D model.
//!
//! \tparam scalar_type The type of the model's vector components.
template <typename scalar_type>
class model {
  //! A type definition for a triangle used by the model.
  using triangle_type = triangle<scalar_type>;
  //! The triangles from the .obj file.
  std::vector<triangle_type> triangles;
public:
  //! Loads a model from a file.
  //!
  //! \param filename The path to the file to load.
  //! This only supports .obj files.
  //!
  //! \return True on success, false on failure.
  bool load(const char* filename) {

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(filename)) {
      return false;
    }

    const auto& attrib = reader.GetAttrib();

    // Unpack face indices

    const auto& shapes = reader.GetShapes();

    for (const auto& shape : shapes) {

      for (std::size_t i = 2; i < shape.mesh.indices.size(); i += 3) {

        size_type v[3] {
          size_type(shape.mesh.indices[i - 2].vertex_index),
          size_type(shape.mesh.indices[i - 1].vertex_index),
          size_type(shape.mesh.indices[i - 0].vertex_index)
        };

        size_type vt[3] {
          size_type(shape.mesh.indices[i - 2].texcoord_index),
          size_type(shape.mesh.indices[i - 1].texcoord_index),
          size_type(shape.mesh.indices[i - 0].texcoord_index)
        };

        triangle_type t {
          {
            {
              scalar_type(attrib.vertices[(v[0] * 3) + 0]),
              scalar_type(attrib.vertices[(v[0] * 3) + 1]),
              scalar_type(attrib.vertices[(v[0] * 3) + 2]),
            },
            {
              scalar_type(attrib.vertices[(v[1] * 3) + 0]),
              scalar_type(attrib.vertices[(v[1] * 3) + 1]),
              scalar_type(attrib.vertices[(v[1] * 3) + 2])
            },
            {
              scalar_type(attrib.vertices[(v[2] * 3) + 0]),
              scalar_type(attrib.vertices[(v[2] * 3) + 1]),
              scalar_type(attrib.vertices[(v[2] * 3) + 2])
            }
          },
          {
            {
              scalar_type(attrib.texcoords[(vt[0] * 2) + 0]),
              scalar_type(attrib.texcoords[(vt[0] * 2) + 1])
            },
            {
              scalar_type(attrib.texcoords[(vt[1] * 2) + 0]),
              scalar_type(attrib.texcoords[(vt[1] * 2) + 1])
            },
            {
              scalar_type(attrib.texcoords[(vt[2] * 2) + 0]),
              scalar_type(attrib.texcoords[(vt[2] * 2) + 1])
            }
          }
        };

        triangles.emplace_back(t);
      }
    }

    return true;
  }
  //! Accesses a pointer to the triangle data.
  const auto* data() const noexcept {
    return triangles.data();
  }
  //! Indicates the number of triangles in the model.
  size_type size() const noexcept {
    return triangles.size();
  }
};

//! Used for converting triangles in the model to bounding boxes.
//!
//! \tparam scalar_type The scalar type of the bounding box vectors to make.
template <typename scalar_type>
class triangle_aabb_converter final {
public:
  //! A type definition for a model bouding box.
  using box_type = aabb<scalar_type>;
  //! A type definition for a triangle.
  using triangle_type = triangle<scalar_type>;
  //! Gets a bounding box for a triangle in the model.
  //!
  //! \param t The triangle to get the bounding box for.
  //!
  //! \return The bounding box for the specified triangle.
  box_type operator () (const triangle_type& t) const noexcept {

    auto tmp_min = math::min(t.pos[0], t.pos[1]);
    auto tmp_max = math::max(t.pos[0], t.pos[1]);

    return box_type {
      math::min(tmp_min, t.pos[2]),
      math::max(tmp_max, t.pos[2])
    };
  }
};

//! Used to detect intersections between rays and triangles.
//!
//! \tparam scalar_type The scalar type of the triangle vector components.
template <typename scalar_type>
class triangle_intersector final {
public:
  //! A type definition for a 2D vector.
  using vec2_type = vec2<scalar_type>;
  //! A type definition for a triangle.
  using triangle_type = triangle<scalar_type>;
  //! A type definition for an intersection.
  using intersection_type = intersection<scalar_type>;
  //! A type definition for a ray.
  using ray_type = ray<scalar_type>;
  //! Detects intersection between a ray and the triangle.
  intersection_type operator () (const triangle<scalar_type>& tri, const ray_type& r) const noexcept {

    using namespace lbvh::math;

    // Basic Möller and Trumbore algorithm

    auto v0v1 = tri.pos[1] - tri.pos[0];
    auto v0v2 = tri.pos[2] - tri.pos[0];

    auto pvec = cross(r.dir, v0v2);

    auto det = dot(v0v1, pvec);

    if (std::fabs(det) < std::numeric_limits<scalar_type>::epsilon()) {
      return intersection_type{};
    }

    auto inv_det = scalar_type(1) / det;

    auto tvec = r.pos - tri.pos[0];

    auto u = dot(tvec, pvec) * inv_det;

    if ((u < 0) || (u > 1)) {
      return intersection_type{};
    }

    auto qvec = cross(tvec, v0v1);

    auto v = dot(r.dir, qvec) * inv_det;

    if ((v < 0) || (u + v) > 1) {
      return intersection_type{};
    }

    auto t = dot(v0v2, qvec) * inv_det;
    if (t < std::numeric_limits<scalar_type>::epsilon()) {
      return intersection_type{};
    }

    // At this point, we know we have a hit.
    // We just need to calculate the UV coordinates.

    vec2_type uv = (tri.uv[0] * (scalar_type(1.0) - u - v)) + (tri.uv[1] * u) + (tri.uv[2] * v);

    return intersection_type {
      t, { 0, 0, 1 }, { uv.x, uv.y }, 0
    };
  }
};

} // namespace lbvh
