#include <lbvh.h>

#include "models/model.h"

#include <chrono>

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

using namespace lbvh;

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

  return errors ? EXIT_FAILURE : EXIT_SUCCESS;
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
