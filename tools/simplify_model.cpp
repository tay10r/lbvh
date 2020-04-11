#include "third-party/tiny_obj_loader.h"

#include "lbvh.h"

//! \brief Converts a .obj file into a simplified list of triangles.
//!
//! \tparam scalar_type The scalar type to export the triangles as.
//!
//! \param input The input filename.
//!
//! \param output The output file name.
//!
//! \return True on success, false on failure.
template <typename scalar_type>
bool simplify(const char* input, const char* output) {

  using vec3_type = lbvh::vec3<scalar_type>;
  using vec2_type = lbvh::vec2<scalar_type>;

  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(input)) {
    return false;
  }

  FILE* output_file = fopen(output, "wb");
  if (!output) {
    return false;
  }

  const auto& attrib = reader.GetAttrib();

  // Unpack face indices

  const auto& shapes = reader.GetShapes();

  for (const auto& shape : shapes) {

    for (std::size_t i = 2; i < shape.mesh.indices.size(); i += 3) {

      int v[3] {
        shape.mesh.indices[i - 2].vertex_index,
        shape.mesh.indices[i - 1].vertex_index,
        shape.mesh.indices[i - 0].vertex_index
      };

      int vt[3] {
        shape.mesh.indices[i - 2].texcoord_index,
        shape.mesh.indices[i - 1].texcoord_index,
        shape.mesh.indices[i - 0].texcoord_index
      };

      vec3_type t_pos[3] {
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
      };

      for (int i = 0; i < 3; i++) {
        std::fwrite(&t_pos[i].x, 1, sizeof(scalar_type), output_file);
        std::fwrite(&t_pos[i].y, 1, sizeof(scalar_type), output_file);
        std::fwrite(&t_pos[i].z, 1, sizeof(scalar_type), output_file);
      }

      vec2_type t_uv[3] {
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
      };

      for (int i = 0; i < 3; i++) {
        std::fwrite(&t_uv[i].x, 1, sizeof(scalar_type), output_file);
        std::fwrite(&t_uv[i].y, 1, sizeof(scalar_type), output_file);
      }
    }
  }

  std::fclose(output_file);

  return true;
}

int main(int argc, char** argv) {

  if (argc < 2) {
    std::fprintf(stderr, "Usage: %s <'.obj' file>\n", argv[0]);
    return EXIT_FAILURE;
  }

  auto success = true;

  success &= simplify<float>(argv[1], "simplified-model-float.bin");
  success &= simplify<double>(argv[1], "simplified-model-double.bin");

  return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
