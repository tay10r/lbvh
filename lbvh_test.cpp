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
      std::printf("  Parent node volume : %8.04f\n", parent_volume);
      std::printf("  Sub node volume    : %8.04f\n", left_volume);
      errors++;
    }
  }

  if (!node.right_is_leaf()) {
    auto right_volume = volume_of(bvh.at(node.right).box);
    if (parent_volume < right_volume) {
      std::printf("Parent node %lu volume is less than right sub node %u\n", index, node.right);
      std::printf("  Parent node volume : %8.04f\n", parent_volume);
      std::printf("  Sub node volume    : %8.04f\n", right_volume);
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

  for (size_type i = 0; i < bvh.size() + 1; i++) {

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

  lbvh::model<float> model;

  std::printf("Loading model: %s\n", filename);

  model.load(filename);

  std::printf("Model loaded\n");

  std::printf("Building BVH\n");

  lbvh::model_aabb_converter<float> aabb_converter(model);

  auto face_indices = model.get_face_indices();

  lbvh::builder<float> builder;

  auto start = std::chrono::high_resolution_clock::now();

  auto bvh = builder(face_indices.data(), face_indices.size(), aabb_converter);

  auto stop = std::chrono::high_resolution_clock::now();

  auto micro_seconds = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

  auto milli_seconds = micro_seconds / 1000.0;

  std::printf("  Completed in %6.03f ms.\n", milli_seconds);

  std::printf("Checking BVH\n");

  auto result = check_bvh(bvh, errors_fatal);

  if (result == EXIT_SUCCESS) {
    std::printf("  Awesomeness! It works.\n");
  }

  return result;
}
