#pragma once

#include <lbvh.h>

#include "tiny_obj_loader.h"

namespace lbvh {

//! Represents an indexed triangular face.
struct face final {
  //! The vertex indices of the face.
  size_type v[3];
  //! The texture coordinate indices of the face.
  size_type vt[3];
};

//! Represents a 3D model.
//!
//! \tparam scalar_type The type of the model's vector components.
template <typename scalar_type>
class model {
  //! A type definition for 2D vectors.
  using vec2_type = vec2<scalar_type>;
  //! A type definition for 3D vectors.
  using vec3_type = vec3<scalar_type>;
  //! The positional vertices of the model.
  std::vector<vec3_type> vertices;
  //! The texture coordinates of the model.
  std::vector<vec2_type> texture_coordinates;
  //! The faces of the model. The faces
  //! from all groups are combined from the file
  //! into this array.
  std::vector<face> faces;
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

    // Unpack vertices

    vertices.resize(attrib.vertices.size() / 3);

    for (size_type i = 2; i < attrib.vertices.size(); i += 3) {

      vec3_type v {
        attrib.vertices[i - 2],
        attrib.vertices[i - 1],
        attrib.vertices[i - 0]
      };

      vertices[i / 3] = v;
    }

    // Unpack texture coordinates

    texture_coordinates.resize(attrib.texcoords.size() / 3);

    for (size_type i = 1; i < attrib.texcoords.size(); i += 2) {

      vec2_type vt {
        attrib.texcoords[i - 1],
        attrib.texcoords[i - 0]
      };

      texture_coordinates.emplace_back(vt);
    }

    // Unpack face indices

    const auto& shapes = reader.GetShapes();

    for (const auto& shape : shapes) {
      for (std::size_t i = 2; i < shape.mesh.indices.size(); i += 3) {

        face f {
          {
            size_type(shape.mesh.indices[i - 2].vertex_index),
            size_type(shape.mesh.indices[i - 1].vertex_index),
            size_type(shape.mesh.indices[i - 0].vertex_index)
          },
          {
            size_type(shape.mesh.indices[i - 2].texcoord_index),
            size_type(shape.mesh.indices[i - 1].texcoord_index),
            size_type(shape.mesh.indices[i - 0].texcoord_index)
          }
        };

        faces.emplace_back(f);
      }
    }

    return true;
  }
  //! Accesses a face from the model.
  //!
  //! \param face_index The index of the face to get.
  //!
  //! \return The face from the specified index.
  inline auto get_face(size_type face_index) const noexcept {
    return faces[face_index];
  }
  //! Accesses a vertex from the model.
  //!
  //! \param vertex The index of the vertex to get.
  //!
  //! \return The vertex at the specified index.
  inline auto get_vertex(size_type vertex) const noexcept {
    return vertices[vertex];
  }
  //! Accesses a vector of indices for each face in the model.
  auto get_face_indices() const {

    std::vector<size_type> indices(faces.size());

    for (size_type i = 0; i < faces.size(); i++) {
      indices[i] = i;
    }

    return indices;
  }
};

//! Used for converting faces in the model to bounding boxes.
//!
//! \tparam scalar_type The scalar type of the bounding box vectors to make.
template <typename scalar_type>
class model_aabb_converter final {
public:
  //! A type definition for the model being used.
  using model_type = model<scalar_type>;
  //! A type definition for a model bouding box.
  using box_type = aabb<scalar_type>;
  //! Constructs a new bounding box converter.
  //!
  //! \param m_ The model to get the bounding boxes for.
  model_aabb_converter(const model_type& m_) : m(m_) {}
  //! Gets a bounding box for a specified face in the model.
  //!
  //! \param face_index The index of the face to get the bounding box for.
  //!
  //! \return The bounding box for the specified face.
  box_type operator () (size_type face_index) const noexcept {

    auto face = m.get_face(face_index);

    auto a = m.get_vertex(face.v[0]);
    auto b = m.get_vertex(face.v[1]);
    auto c = m.get_vertex(face.v[2]);
    
    auto tmp_min = detail::min(a, b);
    auto tmp_max = detail::max(a, b);

    return box_type {
      detail::min(tmp_min, c),
      detail::max(tmp_max, c)
    };
  }
private:
  //! The model to get the bounding boxes for.
  const model_type& m;
};

} // namespace lbvh
