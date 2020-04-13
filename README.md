LBVH
====

**Note:** This project is now read-only. It's been deprecated now that an LBVH algorithm is available from [this library](https://github.com/madmann91/bvh).

This project is an implementation of an LBVH build algorithm,
as described by Karras [here](https://devblogs.nvidia.com/wp-content/uploads/2012/11/karras2012hpg_paper.pdf).

One quality of LBVH trees is that they can be built very quickly.
This makes them an ideal choice for real time ray tracing.

As a single header library, you can easily integrate this into a ray tracing project and speed up your BVH build times!

This project was inspired by the following other open source projects:

 - [Fast-BVH](https://github.com/brandonpelfrey/Fast-BVH)
 - [BVH](https://github.com/madmann91/bvh)
 - [visionaray](https://github.com/szellmann/visionaray)

### Usage

The only requirement the library imposes is that a class be defined which converts primitives to axis-aligned bounding boxes (AABBs). Here's the generally idea in C++ pseudocode.

```cxx
#include <lbvh.h>

struct sphere final {
  float radius;
  float pos[3];
};

int main() {

  sphere spheres[3] {
    { 2, { 3,  5, 6 } },
    { 5, { 7, 10, 1 } },
    { 1, { 0,  1, 2 } }
  };

  auto sphere_to_box = [](const sphere& s) -> lbvh::aabb<float> {
    return lbvh::aabb<float> {
      // min
      {
        s.pos[0] - r,
        s.pos[1] - r,
        s.pos[2] - r
      },
      // max
      {
        s.pos[0] + r,
        s.pos[1] + r,
        s.pos[2] + r
      }
    };
  };

  lbvh::builder<float> builder;

  auto bvh = builder(spheres, 3, sphere_to_box);

  /* Handle BVH here */

  return 0;
}
```
