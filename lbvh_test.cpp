#include <lbvh.h>

#include "models/model.h"

#include "third-party/stb_image_write.h"

#include <chrono>

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

//! Used for size values.
using size_type = lbvh::size_type;

//! The width of the test image.
inline constexpr size_type image_width() noexcept {
  return 1080;
}

//! The height of the test image.
inline constexpr size_type image_height() noexcept {
  return 720;
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
  using vec3_type = lbvh::vec3<scalar_type>;
  //! A type definition for a single ray.
  using ray_type = lbvh::ray<scalar_type>;
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
  void operator () (const lbvh::work_division& div, const trace_kernel& kern, const arg_types&... args) {

    using namespace lbvh::math;

    using channel_type = unsigned char;

    auto cam_dir = normalize(cam_target - cam_pos);
    auto cam_u = normalize(cross(cam_dir, cam_up));
    auto cam_v = normalize(cross(cam_u, cam_dir));

    auto aspect_ratio = scalar_type(x_res) / y_res;

    auto fov = scalar_type(0.75);

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

//! Stores the results of a test.
struct test_results final {
  //! The number of seconds it took to build the BVH.
  double build_time = 0;
  //! The number of seconds it took to render the BVH.
  double render_time = 0;
  //! The generated image buffer.
  std::vector<unsigned char> image_buf;
};

//! Used for getting traits from type.
template <typename scalar_type>
struct type_traits final {};

template <>
struct type_traits<float> {
  static constexpr const char* image_name() noexcept {
    return "test-result-image-float.png";
  }
  static constexpr const char* name() noexcept {
    return "float";
  }
};

template <>
struct type_traits<double> {
  static constexpr const char* image_name() noexcept {
    return "test-result-image-double.png";
  }
  static constexpr const char* name() noexcept {
    return "double";
  }
};

//! A function object that tests the BVH build
//! and traversal algorithm.
//!
//! \tparam scalar_type The scalar type to run the algorithms with.
template <typename scalar_type>
class test final {
  //! A type definition for the builder to be used by the test.
  using builder_type = lbvh::builder<scalar_type>;
  //! A type definition for a BVH.
  using bvh_type = lbvh::bvh<scalar_type>;
  //! A type definition for a single bounding box.
  using box_type = lbvh::aabb<scalar_type>;
  //! The type used for the model that the BVH is being built for.
  using model_type = lbvh::model<scalar_type>;
  //! A type definition for the primitive used in the test.
  using primitive_type = lbvh::triangle<scalar_type>;
  //! A type definition for the class that converts primitives to bounding boxes.
  using converter_type = lbvh::triangle_aabb_converter<scalar_type>;
  //! A type definition for the type used to detect primitive intersections.
  using intersector_type = lbvh::triangle_intersector<scalar_type>;
  //! A type definition for a BVH traverser.
  using traverser_type = lbvh::traverser<scalar_type, primitive_type>;
  //! A type definition for aray.
  using ray_type = lbvh::ray<scalar_type>;
public:
  //! Runs the test.
  //!
  //! \param filename The path to the .obj file to test with.
  //!
  //! \return An instance of @ref test_results containing the relevant data.
  static auto run(const char* filename) {

    std::printf("Running test for type '%s'\n", type_traits<scalar_type>::name());

    model_type model;

    std::printf("  Loading model '%s'\n", filename);

    if (!model.load(filename)) {
      return test_results{};
    }

    std::printf("  Building BVH\n");

    converter_type converter;

    builder_type builder;

    auto build_start = std::chrono::high_resolution_clock::now();

    auto bvh = builder(model.data(), model.size(), converter);

    auto build_stop = std::chrono::high_resolution_clock::now();

    auto build_usecs = std::chrono::duration_cast<std::chrono::microseconds>(build_stop - build_start).count();

    std::printf("  Validating BVH\n");

    if (!check_bvh(bvh, false)) {
      return test_results{};
    }

    std::printf("  Rendering test image.\n");

    auto render_result = render(bvh, model);

    save_image(render_result.first, type_traits<scalar_type>::image_name());

    return test_results {
      build_usecs / 1'000'000.0,
      render_result.second,
      std::move(render_result.first)
    };
  }
protected:
  //! Saves the rendered image to a file.
  //!
  //! \param image The image data to save.
  //!
  //! \param filename The path to save the data to.
  //!
  //! \return True on success, false on failure.
  static bool save_image(const std::vector<unsigned char>& image, const char* filename) {

    // This means RGB format.
    int comp = 3;

    int w = int(image_width());
    int h = int(image_height());

    int stride = w * 3;

    int ret = stbi_write_png(filename, w, h, comp, image.data(), stride);

    return ret == 0;
  }
  //! Renders the model with the built BVH.
  //!
  //! \return An image buffer for the rendered image.
  static auto render(const bvh_type& bvh, const model_type& model) {

    intersector_type intersector;

    traverser_type traverser(bvh, model.data());

    auto tracer_kern = [&traverser, &intersector](const ray_type& r) {

      auto isect = traverser(r, intersector);

      return color<scalar_type> {
        isect.uv.x,
        isect.uv.y,
        0.5
      };
    };

    std::vector<unsigned char> image(image_width() * image_height() * 3);

    ray_scheduler<scalar_type> r_scheduler(image_width(), image_height(), image.data());

    r_scheduler.move_cam({ -1000, 1000, 0 });

    lbvh::default_scheduler thread_scheduler;

    auto trace_start = std::chrono::high_resolution_clock::now();

    thread_scheduler(r_scheduler, tracer_kern);

    auto trace_stop = std::chrono::high_resolution_clock::now();

    auto trace_usecs = std::chrono::duration_cast<std::chrono::microseconds>(trace_stop - trace_start).count();

    auto trace_time = trace_usecs / 1'000'000.0;

    return std::pair<decltype(image), double>(std::move(image), trace_time);
  }
  //! \brief This function validates the BVH that was built,
  //! ensuring that all leafs get referenced once and all nodes
  //! other than the root node get referenced once as well.
  //!
  //! \param bvh The BVH to validate.
  //!
  //! \param errors_fatal If true, the first error causes
  //! the function to return.
  //!
  //! \return True on success, false on failure.
  static bool check_bvh(const bvh_type& bvh, bool errors_fatal) {

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
        return false;
      }
    }

    for (size_type i = 1; i < node_counts.size(); i++) {

      auto n = node_counts[i];

      if (n != 1) {
        std::printf("%s:%d: Node %lu was counted %lu times.\n", __FILE__, __LINE__, i, n);
        if (errors_fatal) {
          return false;
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
          return false;
        } else {
          errors++;
        }
      }
    }

    if (errors) {
      return false;
    } else {
      return check_volumes(bvh, errors_fatal);
    }
  }
  //! Checks the volumes of a BVH,
  //! ensuring that all sub nodes have a volume that's smaller than their parent.
  //!
  //! \param errors_fatal Whether or not the function should exit
  //! at the first occurence of an error.
  //!
  //! \param index The index of the node to check. Since this is
  //! a recursive function, this parameter is only set on recursive calls.
  //!
  //! \return True on success, false on failure.
  static bool check_volumes(const bvh_type& bvh, bool errors_fatal, size_type index = 0) {

    const auto& node = bvh.at(index);

    auto parent_volume = volume_of(node.box);

    int errors = 0;

    if (!node.left_is_leaf()) {
      auto left_volume = volume_of(bvh.at(node.left).box);
      if (parent_volume < left_volume) {
        std::printf("Parent node %u volume is less than left sub node %u\n", unsigned(index), unsigned(node.left));
        std::printf("  Parent node volume : %8.04f\n", double(parent_volume));
        std::printf("  Sub node volume    : %8.04f\n", double(left_volume));
        errors++;
      }
    }

    if (!node.right_is_leaf()) {
      auto right_volume = volume_of(bvh.at(node.right).box);
      if (parent_volume < right_volume) {
        std::printf("Parent node %u volume is less than right sub node %u\n", unsigned(index), unsigned(node.right));
        std::printf("  Parent node volume : %8.04f\n", double(parent_volume));
        std::printf("  Sub node volume    : %8.04f\n", double(right_volume));
        errors++;
      }
    }

    if (errors && errors_fatal) {
      return false;
    }

    bool exit_code = !errors;

    if (!node.left_is_leaf()) {
      auto ret = check_volumes(bvh, errors_fatal, node.left);
      if (!ret) {
        if (errors_fatal) {
          return false;
        } else {
          exit_code = ret;
        }
      }
    }

    if (!node.right_is_leaf()) {
      auto ret = check_volumes(bvh, errors_fatal, node.right);
      if (!ret) {
        exit_code = ret;
      }
    }

    return exit_code;
  }
  //! \brief Calculates the volume of a bounding box.
  //! This is used to compare the volume of bounding
  //! boxes, between the parent and sub nodes.
  static scalar_type volume_of(const box_type& box) noexcept {
    auto size = lbvh::detail::size_of(box);
    return size.x * size.y * size.z;
  }
};

} // namespace

#ifndef MODEL_PATH
#define MODEL_PATH "models/sponza.obj"
#endif

int main() {

  std::vector<test_results> results;

  const char* model_path = MODEL_PATH;

  results.emplace_back(test<float>::run(model_path));
  results.emplace_back(test<double>::run(model_path));

  std::printf("\n");

  const char* type_names[] = {
    " float     ",
    " double    "
  };

  std::printf("Summary of test results:\n");
  std::printf("\n");
  std::printf("| Scalar Type | Build Time | Render Time |\n");
  std::printf("|-------------|------------|-------------|\n");

  for (size_type i = 0; i < results.size(); i++) {
    std::printf("| %s | %9.08f | %10.09f |\n",
                type_names[i],
                double(results[i].build_time),
                double(results[i].render_time));
  }

  std::printf("\n");

  for (size_type i = 1; i < results.size(); i++) {

    long total_diff = 0;

    for (size_type j = 0; j < results[0].image_buf.size(); j++) {

      long channel_diff = 0;
      channel_diff += long(results[0].image_buf[j]);
      channel_diff -= long(results[i].image_buf[j]);

      channel_diff = channel_diff < 0 ? -channel_diff
                                      :  channel_diff;

      total_diff += channel_diff;
    }

    long max_diff = 255L * long(results[0].image_buf.size());

    double percent_diff = 100.0 * double(total_diff) / double(max_diff);

    std::printf("Image results 0 and %u differ by %%%.06f\n",
                unsigned(i), percent_diff);
  }

  return 0;
}
