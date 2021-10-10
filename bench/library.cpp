#include "library.h"

#include "bvh_library.h"
#include "this_library.h"

std::unique_ptr<library>
library::create_library(which w)
{
  switch (w) {
    case which::lbvh:
      return std::unique_ptr<library>(new this_library());
    case which::bvh_sweep_sah:
      return std::unique_ptr<library>(
        new bvh_library(bvh_library::builder::sweep_sah));
    case which::bvh_lbvh:
      return std::unique_ptr<library>(
        new bvh_library(bvh_library::builder::lbvh));
  }

  return nullptr;
}
