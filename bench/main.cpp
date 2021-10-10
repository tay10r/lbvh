#include "library.h"

#include "../third-party/stb_image_write.h"
#include "../third-party/tiny_obj_loader.h"

#include <chrono>
#include <map>
#include <string>
#include <vector>

#include <cstdio>
#include <cstdlib>

namespace {

std::vector<float>
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

  std::vector<float> vertices = open_obj(argv[1]);

  const float* verts = vertices.data();

  const size_t tri_count = vertices.size() / 9;

  const size_t w = 3840;
  const size_t h = 2160;

  using library_ptr = std::unique_ptr<library>;

  std::map<std::string, library_ptr> library_map;

  library_map["lbvh"] = library::create_library(library::which::lbvh);

  library_map["bvh(sweep_sah)"] =
    library::create_library(library::which::bvh_sweep_sah);

  library_map["bvh(lbvh)"] = library::create_library(library::which::bvh_lbvh);

  for (const auto& library_entry : library_map) {

    std::printf("testing '%s'\n", library_entry.first.c_str());

    std::vector<float> rgb_buffer(w * h * 3);

    auto t0 = std::chrono::high_resolution_clock::now();

    library_entry.second->build_bvh(verts, tri_count);

    auto t1 = std::chrono::high_resolution_clock::now();

    using milliseconds = std::chrono::milliseconds;

    auto build_time = std::chrono::duration_cast<milliseconds>(t1 - t0).count();

    std::printf("  bvh build took %lu ms\n", (unsigned long int)build_time);

    t0 = std::chrono::high_resolution_clock::now();

    library_entry.second->render(&rgb_buffer[0], w, h);

    t1 = std::chrono::high_resolution_clock::now();

    auto render_time =
      std::chrono::duration_cast<milliseconds>(t1 - t0).count();

    std::printf("  render took %lu ms\n", (unsigned long int)render_time);

    save_png(library_entry.first + "_result.png", rgb_buffer, w, h);
  }

  return EXIT_SUCCESS;
}

namespace {

std::vector<float>
open_obj(const char* path)
{
  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(path)) {
    std::fprintf(stderr, "Failed to open '%s'\n", path);
    std::exit(EXIT_FAILURE);
  }

  size_t vertex_count = 0;

  for (const auto& shape : reader.GetShapes())
    vertex_count += shape.mesh.indices.size();

  std::vector<float> vertices(vertex_count * 3);

  size_t vertex_index = 0;

  const auto& attrib = reader.GetAttrib();

  for (const auto& shape : reader.GetShapes()) {

    for (size_t i = 0; i < shape.mesh.indices.size(); i++) {

      const int offset = shape.mesh.indices[i + 0].vertex_index * 3;

      const float* vertex_data = &attrib.vertices[offset];

      vertices[vertex_index + 0] = vertex_data[0];
      vertices[vertex_index + 1] = vertex_data[1];
      vertices[vertex_index + 2] = vertex_data[2];

      vertex_index += 3;
    }
  }

  return vertices;
}

bool
save_png(const std::string& path,
         const std::vector<float>& rgb,
         size_t w,
         size_t h)
{
  std::vector<unsigned char> rgb888(rgb.size() * 3);

  for (size_t i = 0; i < (w * h * 3); i++)
    rgb888[i] = std::min(std::max(rgb[i] * 255.0f, 0.0f), 255.0f);

  return stbi_write_png(path.c_str(), w, h, 3, &rgb888[0], w * 3);
}

} // namespace
