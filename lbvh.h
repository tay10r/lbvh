//  The MIT License (MIT)
//
// Copyright (c) 2020 Taylor Holberton and contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

//! @file lbvh.h LBVH Header
//!
//! @brief This header contains all the code required to build
//! and traverse an LBVH. It uses templates all throughout the
//! code base and is therefore easily optimized by compilers and
//! easily extended to multiple floating point types and primitive types.
//!
//! See @ref lbvh::builder on building an BVH from this library.
//! It will require you to write a lambda functor or function object
//! that computes the bounding box of your primitive. Primitives themselves
//! are defined all throughout the code as template types, so they can be anything.
//!
//! As an extra bit of functionality, there's also a BVH traverser
//! class bundled with this header. See @ref lbvh::traverser for details.
//! It will require you to write a lambda functor or a function object
//! that computes an intersection between a ray and your primitive type.

#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#if (__cplusplus >= 201703L) && !(defined LBVH_NO_THREADS)
#include <execution>
#endif

#include <cmath>
#include <cstdint>

namespace lbvh {

//! This is the type used for size values.
using size_type = std::size_t;

//! This class is used to associate certain
//! types with a type size.
template <size_type type_size>
struct associated_types final { };

//! \brief This class is used to describe
//! the amount of work that a task is assigned.
struct work_division final {
  //! The starting index of the work amount.
  size_type idx;
  //! The maximum divisions of the work.
  size_type max;
};

//! \brief This is a task scheduler that schedules
//! only the current thread for work. It's
//! a placeholder if the user doesn't specifier
//! their own thread pool library.
class sync_task_scheduler final {
public:
  //! Issues a new task to be performed.
  //! In this class, the task immediately is called in the current thread.
  template <typename task_type, typename... arg_types>
  inline void operator () (task_type task, arg_types... args) noexcept {
    task(work_division { 0, 1 }, args...);
  }
  //! Indicates the maximum number of threads
  //! to be invoked at a time.
  inline size_type max_threads() const noexcept {
    return 1;
  }
};

//! \brief Calculates the highest bit for an integer type.
//!
//! \return An integer with the highest bit set to one.
template <typename int_type>
inline constexpr int_type highest_bit() noexcept {
  return 1 << ((sizeof(int_type) * 8) - 1);
}

//! Represents a single 2D vector.
//!
//! \tparam scalar_type The type of the vector components.
template <typename scalar_type>
struct vec2 final {
  //! The X component of the vector.
  scalar_type x;
  //! The Y component of the vector.
  scalar_type y;
};

//! Represents a single 3D vector.
//!
//! \tparam scalar_type The type of the vector components.
template <typename scalar_type>
struct vec3 final {
  //! The X component of the vector.
  scalar_type x;
  //! The Y component of the vector.
  scalar_type y;
  //! The Z component of the vector.
  scalar_type z;
};

//! Represents a single axis-aligned bounding box.
//! \tparam scalar_type The scalar type used for vector values.
template <typename scalar_type>
struct aabb final {
  //! The minimum points of the box.
  vec3<scalar_type> min;
  //! The maximum points of the box.
  vec3<scalar_type> max;
};

//! An internal node within the BVH.
//! Points to two other nodes, which
//! may either be leaf nodes or other internal nodes.
//!
//! \tparam scalar_type The type used for the bounding
//! box vectors of the node.
template <typename scalar_type>
struct alignas(sizeof(scalar_type) * 8) node final {
  //! The type definition for a node index.
  using index_type = typename associated_types<sizeof(scalar_type)>::uint_type;
  //! The box that contains the node.
  aabb<scalar_type> box;
  //! The index to the left node.
  //! The highest bit may be set to
  //! one if this is suppose to point to a leaf.
  index_type left;
  //! The index to the right node.
  index_type right;
  //! The highest bit may be set to
  //! one if this is suppose to point to a leaf.
  //! Accesses the left index as a leaf index.
  inline constexpr index_type left_leaf_index() const noexcept {
    return left & (highest_bit<index_type>() - 1);
  }
  //! Accesses the right index as a leaf index.
  inline constexpr index_type right_leaf_index() const noexcept {
    return right & (highest_bit<index_type>() - 1);
  }
  //! Indicates if the left index points to a leaf.
  inline constexpr bool left_is_leaf() const noexcept {
    return left & highest_bit<index_type>();
  }
  //! Indicates if the right index points to a leaf.
  inline constexpr bool right_is_leaf() const noexcept {
    return right & highest_bit<index_type>();
  }
};

//! This is the structure for the LBVH.
//! \tparam float_type The floating point type used for box vectors.
template <typename float_type>
class bvh final {
public:
  //! A type definition for BVH nodes.
  using node_type = node<float_type>;
  //! A type definition for a BVH node vector.
  using node_vec = std::vector<node_type>;
  //! Constructs a BVH from prebuilt internal nodes.
  bvh(node_vec&& nodes_) : nodes(std::move(nodes_)) {}
  //! Accesses the beginning iterator.
  inline auto begin() const noexcept { return nodes.begin(); }
  //! Accesses the ending iterator.
  inline auto end() const noexcept { return nodes.end(); }
  //! Indicates the number of internal nodes in the BVH.
  inline auto size() const noexcept { return nodes.size(); }
  //! Accesses a reference to a node within the BVH.
  //! An exception is thrown if the index is out of bounds.
  //! To access nodes without bounds checking, use the [] operator.
  //!
  //! \param index The index of the node to access.
  //!
  //! \return A const-reference to the specified node.
  inline const node_type& at(size_type index) const {
    return nodes.at(index);
  }
  //! Accesses a reference to a node within the BVH.
  //! This function does not perform bounds checking.
  //!
  //! \param index The index of the node to access.
  //!
  //! \returns A const-reference to the box type.
  inline const node_type& operator [] (size_type index) const noexcept {
    return nodes[index];
  }
private:
  //! The internal nodes of the BVH.
  node_vec nodes;
};

//! \brief This class is used for the constructing of BVHs.
//! It is meant to be the first class declared by anyone using the library.
//! It takes a set of primitives, as well as a function object to calculate their bounding boxes,
//! and generates an LBVH from them. The only function required to be implemented is the function
//! object that converts primitives to bounding boxes.
//!
//! \tparam scalar_type The scalar type to use for the box vectors.
template <typename scalar_type, typename task_scheduler = sync_task_scheduler>
class builder final {
  //! Is passed the various work items during BVH construction.
  task_scheduler scheduler;
public:
  //! A type definition for a BVH.
  using bvh_type = bvh<scalar_type>;
  //! A type definition for a BVH node.
  using node_type = node<scalar_type>;
  //! A type definition for a node vector.
  using node_vec = std::vector<node_type>;
  //! Constructs a new BVH builder.
  //! \param scheduler_ The task scheduler to distribute the work with.
  builder(task_scheduler scheduler_ = task_scheduler())
    : scheduler(scheduler_) {}
  //! Builds a BVH from an array of primitives.
  //!
  //! \param primitives The array of primitives to build the BVH for.
  //!
  //! \param count The number of primitives in the primitive array.
  //!
  //! \param converter The primitive to bounding box converter.
  //! This should be implemented by the caller as a function object
  //! that takes a single primitive and returns an instance of @ref aabb
  //! that represents the bounding box for that primitive.
  //!
  //! \return A BVH built for the specified primitives.
  template <typename primitive, typename aabb_converter>
  bvh_type operator () (const primitive* primitives, size_type count, aabb_converter converter);
protected:
  //! Fits BVH nodes with their appropriate boxes.
  template <typename primitive, typename aabb_converter>
  void fit_boxes(node_vec& nodes, const primitive* primitives, aabb_converter converter);
};

//! \brief This structure contains basic information
//! regarding a ray intersection with a BVH. It's not
//! required to be used. Other intersection structures
//! can be used as long as they implement the '<' comparison
//! operator (based on distance) and boolean operator (indicates
//! whether or not a hit occurred.
template <typename scalar_type>
struct alignas(sizeof(scalar_type) * 8) intersection final {
  //! A type definition for an index, used for tracking the intersected primitive.
  using index = typename associated_types<sizeof(scalar_type)>::uint_type;
  //! The distance factor between the ray and primitive.
  scalar_type distance = std::numeric_limits<scalar_type>::max();
  //! The normal at the point of intersection.
  vec3<scalar_type> normal {};
  //! The UV coordinates at the point of intersection.
  vec2<scalar_type> uv {};
  //! The index of the primitive that the intersection was made with.
  index primitive = highest_bit<index>();
  //! Indicates whether or not an intersection was made.
  //!
  //! \return True if this intersection has valid intersection data.
  //! False if it does not and no intersection was made.
  operator bool () const noexcept {
    return distance != std::numeric_limits<scalar_type>::max();
  }
  //! Compares two intersection instances.
  //!
  //! \return True if @p a has a distance factor less than @p b.
  bool operator < (const intersection<scalar_type>& other) const noexcept {
    return (distance < other.distance);
  }
};

//! \brief Contains data for a single ray.
//!
//! \tparam scalar_type The type used for the vector components.
template <typename scalar_type>
struct ray final {
  //! A type definition for vectors used by the ray class.
  using vec_type = vec3<scalar_type>;
  //! The starting position of the ray.
  vec_type pos;
  //! The direction at which the ray is pointing at.
  //! Usually, this is not normalized.
  vec_type dir;
  //! The reciprocal of the ray direction.
  vec_type rcp_dir;
  //! Constructs a ray from a position and direction vector.
  //!
  //! \param p The position the ray is being casted from.
  //!
  //! \param d The direction the ray is pointing at.
  //! This does not have to be a normalized vector.
  inline constexpr ray(const vec_type& p,
                       const vec_type& d) noexcept;
};

//! \brief This class is used for traversing a BVH.
//!
//! \tparam scalar_type The floating point type to use for vector components.
//!
//! \tparam primitive The type of the primitive being checked for intersection.
//!
//! \tparam intersection_type The type used for indicating intersections.
template <typename scalar_type,
          typename primitive_type,
          typename intersection_type = intersection<scalar_type>>
class traverser final {
  //! A reference to the BVH being traversed.
  const bvh<scalar_type>& bvh_;
  //! The primitives to check for intersection.
  const primitive_type* primitives;
public:
  //! A type definition for a ray.
  using ray_type = ray<scalar_type>;
  //! Constructs a new traverser instance.
  //! \param b The BVH to be traversed.
  //! \param p The primitives to check for intersection in each box.
  constexpr traverser(bvh<scalar_type>& b, const primitive_type* p) noexcept
    : bvh_(b), primitives(p) {}
  //! \brief Traverses the BVH, returning the closest intersection that was made.
  //!
  //! \tparam intersector_type Defined by the caller as a function object that
  //! takes a primitive and a ray and returns an instance of @ref intersection_type
  //! that indicates whether or not a hit was made.
  template <typename intersector_type>
  intersection_type operator () (const ray_type& ray, intersector_type intersector) const;
};

//! \brief Contains the associated types for 32-bit sizes.
template <>
struct associated_types<4> final {
  //! The unsigned integer type to use for 32-bit types.
  using uint_type = std::uint32_t;
  //! The signed integer type to use for 32-bit types.
  using int_type = std::int32_t;
  //! The floating point type to use for 32-bit types.
  using float_type = float;
};

//! \brief Contains the associated types for 64-bit sizes.
template <>
struct associated_types<8> final {
  //! The unsigned integer type to use for 64-bit types.
  using uint_type = std::uint64_t;
  //! The signed integer type to use for 64-bit types.
  using int_type = std::int64_t;
  //! The floating point type to use for 64-bit types.
  using float_type = double;
};

//! \brief This namespaces contains implementation details
//! of the library. It shouldn't be used by the end-user.
namespace detail {

//! \brief Counts leading zeroes of a 32-bit integer.
inline auto clz(std::uint32_t n) noexcept {
#ifdef _MSC_VER
  return __lzcnt(n);
#else
  return __builtin_clz(n);
#endif
}

//! \brief Counts leading zeroes of a 64-bit integer.
inline auto clz(std::uint64_t n) noexcept {
#ifdef _MSC_VER
  return __lzcnt64(n);
#else
  return __builtin_clzll(n);
#endif
}

//! \brief Calculates the dot product of two 3D vectors.
//!
//! \return The dot product of @p a and @p b.
template <typename scalar_type>
inline constexpr scalar_type dot(const vec3<scalar_type>& a,
                                 const vec3<scalar_type>& b) noexcept {
  scalar_type out = 0;
  out += a.x * b.x;
  out += a.y * b.y;
  out += a.z * b.z;
  return out;
}

//! \brief Normalizes a vector.
//! This is useful when dealing with trig functions.
//!
//! \return The normalized result of @p v.
template <typename scalar_type>
vec3<scalar_type> normalize(const vec3<scalar_type>& v) noexcept {

  using std::sqrt;

  auto l_inv = scalar_type(1) / sqrt(dot(v, v));

  return v * l_inv;
}

//! \brief Calculates the minimum of two scalar values.
template <typename scalar_type>
inline constexpr scalar_type min(scalar_type a, scalar_type b) noexcept {
  return (a < b) ? a : b;
}

//! \brief Calculates the maximum of two scalar values.
template <typename scalar_type>
inline constexpr scalar_type max(scalar_type a, scalar_type b) noexcept {
  return (a > b) ? a : b;
}

//! \brief Gets the sign of a number, as an integer.
//!
//! \return The sign of @p n. If @p n is negative,
//! then negative one is returned. If @p n is greater
//! than or equal to positive zero, then positive one
//! is returned.
template <typename scalar_type>
inline constexpr scalar_type sign(scalar_type n) noexcept {
  return (n < 0) ? -1 : 1;
}

//! Divides and rounds up to the nearest integer.
//!
//! \tparam int_type The integer type to divide with.
//!
//! \return The quotient between @p n and @ d, rounded up.
template <typename int_type>
inline int_type ceil_div(int_type n, int_type d) {
  return (n / d) + ((n % d) ? 1 : 0);
}

//! Used for iterating a range of numbers in a for loop.
struct loop_range final {
  //! The beginning index of the range.
  size_type begin;
  //! The non-inclusive ending index of the range.
  size_type end;
  //! Constructs a loop range for an array and work division.
  //! This makes it easy to divide work required on an array
  //! into a given work division.
  //!
  //! \param div The work division given by the scheduler.
  //!
  //! \param array_size The size of the array being worked on.
  loop_range(const work_division& div, size_type array_size) noexcept {

    auto chunk_size = array_size / div.max;

    begin = chunk_size * div.idx;

    if ((div.idx + 1) == div.max) {
      end = array_size;
    } else {
      end = begin + chunk_size;
    }
  }
};

//! \brief Calculates the Hadamard quotient of two vectors.
//!
//! \tparam scalar_type The scalar type of the vector components.
//!
//! \return The Hadamard quotient of the two vectors.
template <typename scalar_type>
auto hadamard_division(const vec3<scalar_type>& a,
                       const vec3<scalar_type>& b) noexcept {
  return vec3<scalar_type> {
    a.x / b.x,
    a.y / b.y,
    a.z / b.z
  };
}

//! \brief Calculates the minimum between two vectors.
//! This is used frequently in bounding box calculations.
//!
//! \tparam scalar_type The scalar type of the vector components.
//!
//! \return A vector containing the minimum components between @p a and @p b.
template <typename scalar_type>
auto min(const vec3<scalar_type>& a,
         const vec3<scalar_type>& b) noexcept {

  return vec3<scalar_type> {
    std::min(a.x, b.x),
    std::min(a.y, b.y),
    std::min(a.z, b.z)
  };
}

//! \brief Calculates the maximum between two vectors.
//! This is used frequently in bounding box calculations.
//!
//! \tparam scalar_type The scalar type of the vector components.
//!
//! \return A vector containing the maximum components between @p a and @p b.
template <typename scalar_type>
auto max(const vec3<scalar_type>& a,
         const vec3<scalar_type>& b) noexcept {

  return vec3<scalar_type> {
    std::max(a.x, b.x),
    std::max(a.y, b.y),
    std::max(a.z, b.z)
  };
}

//! \brief Calculates a vector with reciprocal values of another vector.
//! This is used for ray-box intersection acceleration.
//!
//! \tparam scalar_type The type used for vector components.
//!
//! \return A vector with reciprocal values of @p in.
template <typename scalar_type>
auto reciprocal(const vec3<scalar_type>& in) noexcept {
  return vec3<scalar_type> {
    1 / in.x,
    1 / in.y,
    1 / in.z
  };
}

//! \brief Calculates the sum of two vectors.
//!
//! \tparam scalar_type The type of the vector components.
//!
//! \return The sum of @p a and @p b, as a vector.
template <typename scalar_type>
auto operator + (const vec3<scalar_type>& a,
                 const vec3<scalar_type>& b) noexcept {
  return vec3<scalar_type> {
    a.x + b.x,
    a.y + b.y,
    a.z + b.z
  };
}

//! \brief Calculates the difference between two vectors.
//!
//! \tparam scalar_type The type of the vector components.
//!
//! \return The difference between @p a and @p b, as a vector.
template <typename scalar_type>
auto operator - (const vec3<scalar_type>& a,
                 const vec3<scalar_type>& b) noexcept {
  return vec3<scalar_type> {
    a.x - b.x,
    a.y - b.y,
    a.z - b.z
  };
}

//! \brief Calculates the product between a vector and a scalar value.
//!
//! \tparam scalar_type The scalar type of the vector.
//!
//! \return The product between @p a and @p b.
template <typename scalar_type>
auto operator * (const vec3<scalar_type>& a, scalar_type b) noexcept {
  return vec3<scalar_type> {
    a.x * b,
    a.y * b,
    a.z * b
  };
}

//! Checks for ray intersection with a bounding box.
//!
//! \tparam scalar_type The type used for vector components.
//!
//! \return True if the ray intersects, false otherwise.
template <typename scalar_type>
auto intersect(const aabb<scalar_type>& box, const ray<scalar_type>& r) noexcept {

  auto tx1 = (box.min.x - r.pos.x) * r.rcp_dir.x;
  auto tx2 = (box.max.x - r.pos.x) * r.rcp_dir.x;

  auto tmin = min(tx1, tx2);
  auto tmax = max(tx1, tx2);

  auto ty1 = (box.min.y - r.pos.y) * r.rcp_dir.y;
  auto ty2 = (box.max.y - r.pos.y) * r.rcp_dir.y;

  tmin = max(tmin, min(ty1, ty2));
  tmax = min(tmax, max(ty1, ty2));

  auto tz1 = (box.min.z - r.pos.z) * r.rcp_dir.z;
  auto tz2 = (box.max.z - r.pos.z) * r.rcp_dir.z;

  tmin = max(tmin, min(tz1, tz2));
  tmax = min(tmax, max(tz1, tz2));

  return (tmax >= max(scalar_type(0), tmin));
}

//! \brief This class represents a space filling curve.
//!
//! \tparam code_type The type used for units of the curve.
template <typename code_type>
class space_filling_curve final {
public:
  //! Represents a single entry within the curve table.
  struct entry final {
    //! The code at this point along the curve.
    code_type code;
    //! The index to the primitive associated with this point.
    size_type primitive;
  };
  //! A type definition for a vector of entries.
  using entry_vec = std::vector<entry>;
  //! Constructs a space filling curve.
  //! \param entries_ The entries to assign the curve.
  space_filling_curve(entry_vec&& entries_)
    : entries(std::move(entries_)) {}
  //! Sorts the space filling curve based on the code of each entry.
  void sort() {

    auto cmp = [](const entry& a, const entry& b) {
      return a.code < b.code;
    };

#if (__cplusplus >= 201703L) && !(defined LBVH_NO_THREADS)
    std::sort(std::execution::par_unseq, entries.begin(), entries.end(), cmp);
#else
    std::sort(entries.begin(), entries.end(), cmp);
#endif
  }
  //! Indicates the number of entries in the space filling curve.
  inline auto size() const noexcept {
    return entries.size();
  }
  //! Accesses a specific entry within the space filling curve.
  //! \param index The index of the entry to access.
  inline const auto& operator [] (size_type index) const noexcept {
    return entries[index];
  }

  space_filling_curve(const space_filling_curve&) = delete;
  space_filling_curve& operator = (const space_filling_curve&) = delete;
private:
  //! The entries of the space filling curve.
  entry_vec entries;
};

//! \brief Calculates the center of a floating-point bounding box.
//!
//! \tparam scalar_type The type of the box vector components.
//! In this function, this is a floating point type.
//!
//! \param box The box to get the center of.
//!
//! \return The center point of the box.
template <typename scalar_type, typename = std::enable_if_t<std::is_floating_point<scalar_type>::value>>
auto center_of(const aabb<scalar_type>& box) noexcept {
  return (box.min + box.max) * scalar_type(0.5);
}

//! \brief Calculates the center of a non-floating-point bounding box.
//!
//! \tparam scalar_type The type of the box vector components.
//! In this function, this type would not be a floating point type.
//!
//! \param box The box to get the center point of.
//!
//! \return The center point of the box.
template <typename scalar_type, std::enable_if_t<!std::is_floating_point<scalar_type>::value>>
auto center_of(const aabb<scalar_type>& box) noexcept {
  return (box.min + box.max) / 2;
}

//! \brief Gets an empty bounding box.
//! An empty box has the property that any union this
//! box has with another box is equal to the other box.
//!
//! \return An empty box.
template <typename scalar_type>
auto get_empty_aabb() noexcept {
  return aabb<scalar_type> {
    {
      std::numeric_limits<scalar_type>::max(),
      std::numeric_limits<scalar_type>::max(),
      std::numeric_limits<scalar_type>::max()
    },
    {
      std::numeric_limits<scalar_type>::min(),
      std::numeric_limits<scalar_type>::min(),
      std::numeric_limits<scalar_type>::min()
    }
  };
}

//! \brief Calculates the union of two bounding boxes.
//! The result is a bounding box that fits both other bounding boxes.
//!
//! \tparam scalar_type The type of the bounding box vector components.
//!
//! \return A bounding box that fits both boxes passed as parameters.
template <typename scalar_type>
auto union_of(const aabb<scalar_type>& a,
              const aabb<scalar_type>& b) noexcept {

  return aabb<scalar_type> {
    min(a.min, b.min),
    max(a.max, b.max)
  };
}

//! \brief Calculates the size of a bounding box.
//! This is considered the change in value from minimum to maximum points.
//!
//! \tparam scalar_type The type of the bounding box vector components.
//!
//! \param box The box to get the size of.
//!
//! \return The size of the given box.
template <typename scalar_type>
auto size_of(const aabb<scalar_type>& box) noexcept {
  return box.max - box.min;
}

//! \brief This class is used for calculating the boundaries of a scene.
//!
//! \tparam scalar_type The scalar type of the bounding box to get.
//!
//! \tparam primitive_type The type of primitive in the scene.
//!
//! \tparam aabb_converter Calculates the bounding box of a primitive.
template <typename scalar_type, typename primitive_type, typename aabb_converter>
class scene_bounds_kernel final {
public:
  //! A type definition for a type returned by this class.
  using box_type = aabb<scalar_type>;
  //! Constructs a new scene bounds kernel.
  //!
  //! \param p The primitive of arrays to get the bounds for.
  //!
  //! \param c The number of primitives in the array.
  //!
  //! \param cvt The primitive to bounding box converter.
  //!
  //! \param th_count The maximum thread count of the scheduler.
  //! This is used to allocate an array for each each thread gets its own AABB.
  scene_bounds_kernel(const primitive_type* p, size_type c, aabb_converter cvt, size_type th_count)
    : primitives(p), count(c), converter(cvt), thread_boxes(th_count) {
  }
  //! Runs the kernel.
  //! Each thread accumulates its own bounding box
  //! for the portion of the scene it was assigned.
  //! When the result is queried, all boxes for each
  //! of the threads are put into union.
  //!
  //! \param div Given by the scheduler to indicate which
  //! portion of the scene this call should get the bounding box of.
  void operator () (const work_division& div) noexcept {

    auto range = loop_range(div, count);

    auto box = get_empty_aabb<scalar_type>();

    for (size_type i = range.begin; i < range.end; i++) {
      box = union_of(box, converter(primitives[i]));
    }

    thread_boxes[div.idx] = box;
  }
  //! After calling the kernel, this function may be used
  //! to get the bounding box of the scene. Internally, this
  //! function will get the union of each box calculated by
  //! each thread issued by the scheduler.
  //!
  //! \return The bounding box of the scene.
  auto get() const noexcept {

    auto scene_box = get_empty_aabb<scalar_type>();

    for (const auto& th_box : thread_boxes) {
      scene_box = union_of(scene_box, th_box);
    }

    return scene_box;
  }
private:
  //! The array of primitives to get the bounding box of.
  const primitive_type* primitives;
  //! The number of primitives in the primitive array.
  size_type count;
  //! The primitive to bounding box converter.
  aabb_converter converter;
  //! The array of boxes, each box allocated
  //! for a thread.
  std::vector<box_type> thread_boxes;
};

//! \brief Used to get the domain of Morton coordinates,
//! based on the size of the type being used.
template <size_type type_size>
struct morton_domain final { };

//! \brief Contains the domain of 32-bit Morton codes.
template <>
struct morton_domain<4> final {
  //! Gets the domain of 32-bit Morton codes.
  static constexpr size_type value() noexcept {
    return 1024;
  }
};

//! \brief Contains the domain of 64-bit Morton codes.
template <>
struct morton_domain<8> final {
  //! Gets the domain of 64-bit Morton codes.
  static constexpr size_type value() noexcept {
    return 1048576;
  }
};

//! \brief This class is used for encoding Morton values.
//! This class is specialized based on the size of a code point.
//!
//! \tparam type_size The type size of a code point.
template <size_type type_size>
class morton_encoder final {};

//! \brief Used for encoding 32-bit Morton codes.
template <>
class morton_encoder<4> final {
public:
  //! A type definition for a 32-bit Morton code.
  using code_type = associated_types<4>::uint_type;
  //! Encodes a 3D 32-bit Morton code.
  inline code_type operator () (code_type x, code_type y, code_type z) noexcept {
    return (expand(x) << 2)
         | (expand(y) << 1)
         | (expand(z) << 0);
  };
protected:
  //! Expands a 10-bit value to 30-bits.
  inline static code_type expand(code_type n) noexcept {
    n = (n | (n << 16)) & 0x030000ff;
    n = (n | (n << 8)) & 0x0300f00f;
    n = (n | (n << 4)) & 0x030c30c3;
    n = (n | (n << 2)) & 0x09249249;
    return n;
  }
};

//! \brief Used for encoding 64-bit Morton codes.
template <>
class morton_encoder<8> final {
public:
  //! A type definition for a 64-bit Morton code.
  using code_type = associated_types<8>::uint_type;
  //! Encodes a 3D 64-bit Morton code.
  inline code_type operator () (code_type x, code_type y, code_type z) noexcept {
    return (expand(x) << 2)
         | (expand(y) << 1)
         | (expand(z) << 0);
  };
protected:
  //! Expands a 20-bit value to 60-bits.
  inline static code_type expand(code_type n) noexcept {
    n = (n | n << 32) & 0x001f00000000ffff;
    n = (n | n << 16) & 0x001f0000ff0000ff;
    n = (n | n << 8) & 0x100f00f00f00f00f;
    n = (n | n << 4) & 0x10c30c30c30c30c3;
    n = (n | n << 2) & 0x1249249249249249;
    return n;
  }
};

//! This class is used for computing part or all
//! of a Morton curve. It may be called from multiple threads.
//!
//! \tparam scalar_type The type of scalar used in the scene primitives.
//!
//! \tparam primitive_type The type of primitive in the scene.
template <typename scalar_type, typename primitive_type>
class morton_curve_kernel final {
public:
  //! A type definition for a code value.
  using code_type = typename associated_types<sizeof(scalar_type)>::uint_type;
  //! A type definition for an entry in the space filling curve.
  using entry = typename space_filling_curve<code_type>::entry;
  //! Constructs a new Morton curve kernel.
  //! \param p The primitive array to generate the values from.
  //! \param e The entry array to receive the values.
  //! \param c The number of primitives in the array.
  constexpr morton_curve_kernel(const primitive_type* p, entry* e, size_type c) noexcept
    : primitives(p), entries(e), count(c) {}
  //! Calculates the Morton codes of a certain subset of the scene.
  //! The amount of work that's done depends on the work division.
  //!
  //! \param div Passed by the scheduler to indicate the amount of work to do.
  //!
  //! \param scene_box The bounding box for the scene.
  //!
  //! \param converter The primitive to bounding box converter.
  template <typename aabb_converter>
  void operator () (const work_division& div, const aabb<scalar_type>& scene_box, aabb_converter converter) {

    auto mdomain = scalar_type(morton_domain<sizeof(scalar_type)>::value());

    morton_encoder<sizeof(code_type)> encoder;

    auto scene_size = size_of(scene_box);

    auto range = loop_range(div, count);

    for (size_type i = range.begin; i < range.end; i++) {

      auto center = center_of(converter(primitives[i]));

      // Normalize to bounded interval: 0 < x < 1

      auto normalized_center = hadamard_division((center - scene_box.min), scene_size);

      // Convert to point in "Morton space"

      auto morton_center = normalized_center * mdomain;

      auto code = encoder(morton_center.x, morton_center.y, morton_center.z);

      entries[i] = entry { code, i };
    }
  }
private:
  //! The primitives the curve is being generated from.
  const primitive_type* primitives;
  //! The entries to receive the calculated values.
  entry* entries;
  //! The number of primitives in the scene.
  //! This is also the number of entries.
  size_type count;
};

//! \brief This class is used for generating Morton curves.
//!
//! \tparam scalar_type The type for the 3D points
//! that get converted into Morton curves.
//!
//! \tparam task_scheduler The task scheduler to pass workers to.
template <typename scalar_type, typename task_scheduler>
class morton_curve_builder final {
  //! A reference to the task scheduler to use.
  //! This is most likely coming straight from the copy
  //! used by the BVH builder.
  task_scheduler& scheduler;
public:
  //! A type definition for a Morton curve.
  using code_type = typename associated_types<sizeof(scalar_type)>::uint_type;
  //! A type definition for the curve generated by this class.
  using curve_type = space_filling_curve<code_type>;
  //! Constructs a new curve builder.
  //! \param scheduler_ The scheduler to pass work items to.
  morton_curve_builder(task_scheduler& scheduler_)
    : scheduler(scheduler_) {}
  //! Converts a set of primitives into a space filling curve.
  //!
  //! \param primitives The array of primitives to convert.
  //!
  //! \param count The number of primitives in the primitive array.
  //!
  //! \param converter The primitive to bounding box converter.
  //!
  //! \return A space filling curve with Morton codes.
  template <typename primitive, typename aabb_converter>
  curve_type operator () (const primitive* primitives, size_type count, aabb_converter converter) {

    using entry_vec = typename curve_type::entry_vec;

    using scene_bounds_kernel_type = scene_bounds_kernel<scalar_type, primitive, aabb_converter>;

    entry_vec entries(count);

    scene_bounds_kernel_type scene_bounds_kern(primitives, count, converter, scheduler.max_threads());

    scheduler(scene_bounds_kern);

    auto scene_box = scene_bounds_kern.get();

    morton_curve_kernel<scalar_type, primitive> kernel(primitives, entries.data(), count);

    scheduler(kernel, scene_box, converter);

    return curve_type(std::move(entries));
  }
};

//! \brief Represents a division of an internal LBVH node.
struct node_division final {
  //! The first index of the division.
  size_type index_a;
  //! The second index of the division.
  size_type index_b;
  //! The index at which the division occurs.
  size_type split;
  //! Accesses the minimum index of the division.
  inline constexpr auto min() const noexcept {
    return (index_a < index_b) ? index_a : index_b;
  }
  //! Accesses the maximum index of the division.
  inline constexpr auto max() const noexcept {
    return (index_b > index_a) ? index_b : index_a;
  }
};

//! This function divides an internal node into two ranges.
//!
//! \tparam code_type The type used for codes in the space filling curve.
//!
//! \param table The space filling curve, used to determine the indices of the split.
//!
//! \param node_index The index of the node being divided.
//!
//! \return A division structure instance, which may be used to assign sub nodes.
template <typename code_type>
node_division divide_node(const space_filling_curve<code_type>& table, size_type node_index) noexcept {

  // Used as the return value of the delta operator.
  using delta_type = typename associated_types<sizeof(code_type)>::int_type;

  // The type used for offsets.
  using offset_type = typename associated_types<sizeof(code_type)>::int_type;

  // Calculates upper bits, or returns -1
  // if 'k' is out of bounds. Appears in the
  // LBVH paper as 'δ' (lower case delta)
  auto calc_delta = [&table](int j, int k) -> delta_type {

    if ((k < 0) || (k >= int(table.size()))) {
      // Suggested by the paper for out of bounds 'k'
      return -1;
    }

    auto l_code = table[j].code;
    auto r_code = table[k].code;

    if (l_code == r_code) {
      // Suggested by the paper in
      // case the code are equal.
      return clz(uint32_t(j ^ k)) + 32;
    } else {
      return clz(uint32_t(l_code ^ r_code));
    }
  };

  // It's easier in this function to regard
  // all indices as signed offset types.
  auto i = offset_type(node_index);

  auto table_size = offset_type(table.size());

  auto d = sign(calc_delta(i, i + 1) - calc_delta(i, i - 1));

  auto delta_min = calc_delta(i, i - d);

  offset_type l_max = 128;

  while (true) {

    auto k = i + (l_max * d);

    if (k >= table_size) {
      break;
    }

    if (!(calc_delta(i, k) > delta_min)) {
      break;
    }

    l_max *= 4;
  }

  offset_type l = 0;

  for (offset_type div = 2; true; div *= 2) {

    auto t = l_max / div;

    auto k = i + ((l + t) * d);

    if (calc_delta(i, k) > delta_min) {
      l += t;
    }

    if (t == 1) {
      break;
    }
  }

  auto j = i + (l * d);

  auto delta_node = calc_delta(i, j);

  offset_type s = 0;

  for (offset_type div = 2; true; div *= 2) {

    auto t = ceil_div(l, div);

    auto k = i + ((s + t) * d);

    if (calc_delta(i, k) > delta_node) {
      s += t;
    }

    if (div >= l) {
      break;
    }
  }

  auto gamma = i + (s * d) + min(d, 0);

  return node_division {
    size_type(i),
    size_type(j),
    size_type(gamma)
  };
}

//! \brief Used for building the BVH nodes.
//! Can be called by the scheduler from many threads.
//!
//! \tparam code_type The type of code contained by the space filling curve.
//!
//! \tparam scalar_type The scalar type used by the node boxes.
template <typename code_type, typename scalar_type>
class builder_kernel final {
public:
  //! A type definition for a space filling curve.
  using curve_type = space_filling_curve<code_type>;
  //! A type definition for a node type.
  using node_type = node<scalar_type>;
  //! Constructs a new builder kernel.
  //! \param c The curve containing the codes to build the nodes with.
  //! \param n The allocated node array to put the node data into.
  constexpr builder_kernel(const curve_type& c, node_type* n) noexcept
    : curve(c), nodes(n) {}
  //! Calls the kernel to build a certain portion of the BVH nodes.
  //! \param div The division of work this function call is responsible for.
  void operator () (const work_division& div) noexcept {

    using index_type = typename node_type::index_type;

    auto range = loop_range(div, curve.size() - 1);

    for (auto i = range.begin; i < range.end; i++) {

      auto div = divide_node(curve, i);

      auto l_is_leaf = (div.min() == (div.split + 0));
      auto r_is_leaf = (div.max() == (div.split + 1));

      auto l_mask = l_is_leaf ? highest_bit<index_type>() : 0;
      auto r_mask = r_is_leaf ? highest_bit<index_type>() : 0;

      nodes[i].left  = (div.split + 0) | l_mask;
      nodes[i].right = (div.split + 1) | r_mask;
    }
  }
private:
  //! This is the space filling curve used to determine
  //! the range and split of each node that this kernel will build.
  const curve_type& curve;
  //! A pointer to the node array being constructed.
  node_type* nodes;
};

} // namespace detail

template <typename scalar_type>
constexpr ray<scalar_type>::ray(const vec_type& p, const vec_type& d) noexcept
  : pos(p), dir(d), rcp_dir(detail::reciprocal(d)) {}

template <typename scalar_type, typename task_scheduler>
template <typename primitive, typename aabb_converter>
auto builder<scalar_type, task_scheduler>::operator () (const primitive* primitives, size_type count, aabb_converter converter) -> bvh_type {

  using node_type = node<scalar_type>;

  using curve_builder_type = detail::morton_curve_builder<scalar_type, task_scheduler>;

  curve_builder_type curve_builder(scheduler);

  auto curve = curve_builder(primitives, count, converter);

  curve.sort();

  std::vector<node_type> node_vec(curve.size() - 1);

  detail::builder_kernel builder_kernel(curve, node_vec.data());

  scheduler(builder_kernel);

  fit_boxes(node_vec, primitives, converter);

  return bvh_type(std::move(node_vec));
};

template <typename scalar_type, typename task_scheduler>
template <typename primitive, typename aabb_converter>
void builder<scalar_type, task_scheduler>::fit_boxes(node_vec& nodes, const primitive* primitives, aabb_converter converter) {

  std::vector<size_type> indices;

  indices.reserve(nodes.size());

  indices.push_back(0);

  for (size_type i = 0; i < indices.size(); i++) {

    auto j = indices[i];

    if (!nodes[j].left_is_leaf()) {
      indices.push_back(nodes[j].left_leaf_index());
    }

    if (!nodes[j].right_is_leaf()) {
      indices.push_back(nodes[j].right_leaf_index());
    }
  }

  for (size_type i = indices.size(); i > 0; i--) {

    auto j = indices[i - 1];

    auto& node = nodes[j];

    if (!node.left_is_leaf()) {
      node.box = nodes[node.left].box;
    } else {
      node.box = converter(primitives[node.left_leaf_index()]);
    }

    if (!node.right_is_leaf()) {
      node.box = detail::union_of(node.box, nodes[node.right].box);
    } else {
      node.box = detail::union_of(node.box, converter(primitives[node.right_leaf_index()]));
    }
  }
}

template <typename scalar_type, typename primitive_type, typename intersection_type>
template <typename intersector_type>
intersection_type traverser<scalar_type, primitive_type, intersection_type>::operator () (const ray_type& ray, intersector_type intersector) const {

  using traversal_queue = std::vector<size_type>;

  traversal_queue node_queue;

  node_queue.push_back(0);

  auto pop = [](traversal_queue& queue) {
    auto i = queue[queue.size() - 1];
    queue.pop_back();
    return i;
  };

  intersection_type closest_isect;

  while (!node_queue.empty()) {

    auto i = pop(node_queue);

    const auto& node = bvh_[i];

    if (!detail::intersect(node.box, ray)) {
      continue;
    }

    intersection_type isect;

    if (node.left_is_leaf()) {
      isect.primitive = node.left_leaf_index();
      isect = intersector(primitives[isect.primitive], ray);
    } else {
      node_queue.push_back(node.left);
    }

    if (node.right_is_leaf()) {
      isect.primitive = node.right_leaf_index();
      isect = intersector(primitives[isect.primitive], ray);
    } else {
      node_queue.push_back(node.right);
    }

    if (!isect) {
      continue;
    }

    if (isect < closest_isect) {
      std::swap(isect, closest_isect);
    }
  }

  return closest_isect;
}

} // namespace lbvh
