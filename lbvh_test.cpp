#include <lbvh.h>

#include "models/model.h"

#include <chrono>

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

using namespace lbvh;

//! \brief Calculates the volume of a bounding box.
//! This is used to compare the volume of bounding
//! boxes, between the parent and sub nodes.
float volume_of(const aabb<float>& box) noexcept {
  auto size = detail::size_of(box);
  return size.x * size.y * size.z;
}

//! \brief This function traverses the box and ensures that every
//! sub node has a box volume that's less than its parent.
//!
//! \param bvh The bvh to check.
//!
//! \param errors_fatal Whether or not the function should exit
//! at the first occurence of an error.
//!
//! \param index The index of the node to check. Since this is
//! a recursive function, this parameter is only set on recursive calls.
int check_bvh_volumes(const bvh<float>& bvh, int errors_fatal, size_type index = 0) {

  const auto& node = bvh.at(index);

  auto parent_volume = volume_of(node.box);

  int errors = 0;

  if (!node.left_is_leaf()) {
    auto left_volume = volume_of(bvh.at(node.left).box);
    if (parent_volume < left_volume) {
      std::printf("Parent node %lu volume is less than left sub node %u\n", index, node.left);
      std::printf("  Parent node volume : %8.04f\n", double(parent_volume));
      std::printf("  Sub node volume    : %8.04f\n", double(left_volume));
      errors++;
    }
  }

  if (!node.right_is_leaf()) {
    auto right_volume = volume_of(bvh.at(node.right).box);
    if (parent_volume < right_volume) {
      std::printf("Parent node %lu volume is less than right sub node %u\n", index, node.right);
      std::printf("  Parent node volume : %8.04f\n", double(parent_volume));
      std::printf("  Sub node volume    : %8.04f\n", double(right_volume));
      errors++;
    }
  }

  if (errors && errors_fatal) {
    return EXIT_FAILURE;
  }

  int exit_code = errors ? EXIT_FAILURE : EXIT_SUCCESS;

  if (!node.left_is_leaf()) {
    int ret = check_bvh_volumes(bvh, errors_fatal, node.left);
    if (ret != EXIT_SUCCESS) {
      if (errors_fatal) {
        return ret;
      } else {
        exit_code = ret;
      }
    }
  }

  if (!node.right_is_leaf()) {
    auto ret = check_bvh_volumes(bvh, errors_fatal, node.right);
    if (ret != EXIT_SUCCESS) {
      exit_code = ret;
    }
  }

  return exit_code;
}

//! \brief This function validates the BVH that was built,
//! ensuring that all leafs get referenced once and all nodes
//! other than the root node get referenced once as well.
//!
//! \param bvh The BVH to validate.
//!
//! \param errors_fatal If non-zero, the first error causes
//! the function to return .
//!
//! \return If non-zero, an error was found within the BVH.
int check_bvh(const bvh<float>& bvh, int errors_fatal) {

  int errors = 0;

  std::vector<size_type> node_counts(bvh.size());

  for (size_type i = 0; i < bvh.size(); i++) {

    if (!bvh[i].left_is_leaf()) {
      node_counts.at(bvh[i].left)++;
    }

    if (!bvh[i].right_is_leaf()) {
      node_counts.at(bvh[i].right)++;
    }
  }

  if (node_counts[0] > 0) {
    std::printf("%s:%d: Root node was referenced %lu times.\n", __FILE__, __LINE__, node_counts[0]);
    if (errors_fatal) {
      return EXIT_FAILURE;
    }
  }

  for (size_type i = 1; i < node_counts.size(); i++) {

    auto n = node_counts[i];

    if (n != 1) {
      std::printf("%s:%d: Node %lu was counted %lu times.\n", __FILE__, __LINE__, i, n);
      if (errors_fatal) {
        return EXIT_FAILURE;
      } else {
        errors++;
      }
    }
  }

  std::vector<size_type> leaf_counts(bvh.size() + 1);

  for (size_type i = 0; i < bvh.size(); i++) {

    if (bvh[i].left_is_leaf()) {
      leaf_counts.at(bvh[i].left_leaf_index())++;
    }

    if (bvh[i].right_is_leaf()) {
      leaf_counts.at(bvh[i].right_leaf_index())++;
    }
  }

  for (size_type i = 0; i < bvh.size() + 1; i++) {
    auto n = leaf_counts[i];
    if (n != 1) {
      std::printf("%s:%d: Leaf %lu was referenced %lu times.\n", __FILE__, __LINE__, i, n);
      if (errors_fatal) {
        return EXIT_FAILURE;
      } else {
        errors++;
      }
    }
  }

  if (errors) {
    return EXIT_FAILURE;
  } else {
    return check_bvh_volumes(bvh, errors_fatal);
  }
}

//! Represents a simple RGB color.
template <typename scalar_type>
struct color final {
  //! The red channel value.
  scalar_type r;
  //! The green channel value.
  scalar_type g;
  //! The blue channel value.
  scalar_type b;
};

//! \brief This class is used for generating rays for the
//! test traversal.
template <typename scalar_type>
class ray_scheduler final {
  //! A type definition for 3D vectors.
  using vec3_type = vec3<scalar_type>;
  //! A type definition for a single ray.
  using ray_type = ray<scalar_type>;
  //! The X resolution of the image to produce.
  size_type x_res;
  //! The Y resolution of the image to produce.
  size_type y_res;
  //! The image buffer to render the samples to.
  unsigned char* image_buf;
  //! The position of the camera.
  vec3_type cam_pos { scalar_type(1.6), scalar_type(1.3), scalar_type(1.6) };
  //! The direction of "up".
  vec3_type cam_up { 0, 1, 0 };
  //! Whether the camera is looking at.
  vec3_type cam_target { 0, 0, 0 };
public:
  //! Constructs a new instance of the ray scheduler.
  ray_scheduler(size_type width, size_type height, unsigned char* buf) noexcept
    : x_res(width), y_res(height), image_buf(buf) { }
  //! Moves the camera to a new location.
  void move_cam(const vec3_type& v) {
    cam_pos = v;
  }
  //! Executes a kernel across all rays generated from the camera.
  //!
  //! \param kern The ray tracing kernel to pass the rays to.
  template <typename trace_kernel, typename... arg_types>
  void operator () (const work_division& div, const trace_kernel& kern, const arg_types&... args) {

    using namespace lbvh::math;

    using channel_type = unsigned char;

    auto cam_dir = normalize(cam_target - cam_pos);
    auto cam_u = normalize(cross(cam_dir, cam_up));
    auto cam_v = normalize(cross(cam_u, cam_dir));

    auto aspect_ratio = scalar_type(x_res) / y_res;

    auto fov = 0.75f;

    for (size_type y = div.idx; y < y_res; y += div.max) {

      auto* pixels = image_buf + (y * x_res * 3);

      for (size_type x = 0; x < x_res; x++) {

        auto x_ndc =  (2 * (x + scalar_type(0.5)) / scalar_type(x_res)) - 1;
        auto y_ndc = -(2 * (y + scalar_type(0.5)) / scalar_type(y_res)) + 1;

        ray_type r {
          cam_pos,
          normalize((cam_u * x_ndc) + (cam_v * y_ndc) + (cam_dir * fov * aspect_ratio))
        };

        auto color = kern(r, args...);

        pixels[0] = channel_type(color.r * 255);
        pixels[1] = channel_type(color.g * 255);
        pixels[2] = channel_type(color.b * 255);

        pixels += 3;
      }
    }
  }
};

//! Runs the test program with a specified floating point type.
//!
//! \tparam scalar_type The floating point type to be used.
//!
//! \param filename The path to the .obj file to render.
//!
//! \param errors_fatal Whether or not to exit on the first error.
//!
//! \return An exit code suitable for the return value of "main."
template <typename scalar_type>
int run_test(const char* filename, int errors_fatal) {

  lbvh::model<scalar_type> model;

  std::printf("Loading model: %s\n", filename);

  model.load(filename);

  std::printf("Model loaded\n");

  std::printf("Building BVH\n");

  lbvh::triangle_aabb_converter<scalar_type> aabb_converter;

  lbvh::triangle_intersector<scalar_type> intersector;

  lbvh::builder<scalar_type> builder;

  auto start = std::chrono::high_resolution_clock::now();

  auto bvh = builder(model.data(), model.size(), aabb_converter);

  auto stop = std::chrono::high_resolution_clock::now();

  auto micro_seconds = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

  auto milli_seconds = micro_seconds / 1000.0;

  std::printf("  Completed in %6.03f ms.\n", milli_seconds);

  std::printf("Checking BVH\n");

  auto result = check_bvh(bvh, errors_fatal);
  if (result != EXIT_SUCCESS) {
    return result;
  }

  std::printf("  Awesomeness! It works.\n");

  std::printf("Traversing resultant BVH.\n");

  traverser<scalar_type, triangle<scalar_type>> traverser(bvh, model.data());

  auto tracer_kern = [&traverser, &intersector](const ray<scalar_type>& r) {

    auto isect = traverser(r, intersector);

    return color<scalar_type> {
      isect.uv.x,
      isect.uv.y,
      0.5
    };
  };

  size_type width = 1080;

  size_type height = 720;

  std::vector<unsigned char> image(width * height * 3);

  ray_scheduler<scalar_type> r_scheduler(width, height, image.data());

  r_scheduler.move_cam({ -1000, 1000, 0 });

  default_scheduler thread_scheduler;

  auto trace_start = std::chrono::high_resolution_clock::now();

  thread_scheduler(r_scheduler, tracer_kern);

  auto trace_stop = std::chrono::high_resolution_clock::now();

  auto trace_micro_seconds = std::chrono::duration_cast<std::chrono::microseconds>(trace_stop - trace_start).count();

  auto trace_milli_seconds = trace_micro_seconds / 1000.0;

  auto rays_per_second = width * height * 1000.0 / trace_milli_seconds;

  std::printf("  Completed in %6.03f ms (%5.02fM primary rays per second).\n", trace_milli_seconds, rays_per_second / 1'000'000.0);

  std::string ofilename("test-result-float-");
  ofilename += std::to_string(sizeof(scalar_type) * 8);
  ofilename += ".ppm";

  FILE* file = std::fopen(ofilename.c_str(), "wb");
  if (!file) {
    std::fprintf(stderr, "Failed to open '%s'\n", ofilename.c_str());
    return EXIT_FAILURE;
  }

  std::fprintf(file, "P6\n%lu %lu\n%lu\n", width, height, 255UL);

  std::fwrite(image.data(), image.size(), 1, file);

  std::fclose(file);

  return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char** argv) {

  int errors_fatal = 0;

  const char* filename = "models/sponza.obj";

  for (int i = 1; i < argc; i++) {
    if (std::strcmp(argv[i], "--errors_fatal") == 0) {
      errors_fatal = 1;
    } else if (argv[i][0] != '-') {
      filename = argv[i];
    } else {
      std::fprintf(stderr, "Unknown option '%s'\n", argv[i]);
      return EXIT_FAILURE;
    }
  }

  int result = run_test<float>(filename, errors_fatal);

  return result;
}
