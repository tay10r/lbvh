#include <lbvh.h>

#include "../third-party/tiny_obj_loader.h"

#include <chrono>
#include <map>
#include <string>
#include <vector>

#include <cstdio>
#include <cstdlib>

namespace {

std::vector<lbvh::triangle<float>>
open_obj(const char* path);

bool
save_png(const std::string& path,
         const std::vector<float>& rgb,
         size_t w,
         size_t h);

} // namespace

int
main(int argc, char** argv)
{
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <obj-file>\n", argv[0]);
    return EXIT_FAILURE;
  }

  std::vector<lbvh::triangle<float>> triangles = open_obj(argv[1]);

  lbvh::triangle_aabb_converter<float> aabb_converter;

  lbvh::builder<float> builder;

  auto bvh = builder(triangles.data(), triangles.size(), aabb_converter);

  printf("digraph bvh {\n");

  for (size_t i = 0; i < bvh.size(); i++) {

    const lbvh::node<float>& node = bvh[i];

    const char* l_type = node.left_is_leaf() ? "leaf" : "node";
    const char* r_type = node.right_is_leaf() ? "leaf" : "node";

    const int l_index =
      node.left_is_leaf() ? node.left_leaf_index() : node.left;

    const int r_index =
      node.right_is_leaf() ? node.right_leaf_index() : node.right;

    printf("node_%d -> %s_%d\n", int(i), l_type, l_index);
    printf("node_%d -> %s_%d\n", int(i), r_type, r_index);
  }

  printf("}\n");

  return EXIT_SUCCESS;
}

namespace {

std::vector<lbvh::triangle<float>>
open_obj(const char* path)
{
  using vec3 = lbvh::vec3<float>;

  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(path)) {
    std::fprintf(stderr, "Failed to open '%s'\n", path);
    std::exit(EXIT_FAILURE);
  }

  size_t tri_count = 0;

  for (const auto& shape : reader.GetShapes())
    tri_count += shape.mesh.indices.size() / 3;

  std::vector<lbvh::triangle<float>> triangles(tri_count);

  size_t tri_index = 0;

  const auto& attrib = reader.GetAttrib();

  for (const auto& shape : reader.GetShapes()) {

    for (size_t i = 0; i < shape.mesh.indices.size(); i += 3) {

      const int a = shape.mesh.indices[i + 0].vertex_index * 3;
      const int b = shape.mesh.indices[i + 1].vertex_index * 3;
      const int c = shape.mesh.indices[i + 2].vertex_index * 3;

      const float* a_vert = &attrib.vertices[a];
      const float* b_vert = &attrib.vertices[b];
      const float* c_vert = &attrib.vertices[c];

      const vec3 p0{ a_vert[0], a_vert[1], a_vert[2] };
      const vec3 p1{ b_vert[0], b_vert[1], b_vert[2] };
      const vec3 p2{ c_vert[0], c_vert[1], c_vert[2] };

      triangles[tri_index] = lbvh::triangle<float>(p0, p1, p2);

      tri_index++;
    }
  }

  return triangles;
}

} // namespace
