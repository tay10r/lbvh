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
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
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
//! are defined all throughout the code as template types, so they can be
//! anything.
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

#ifndef LBVH_NO_THREADS
#include <thread>
#endif

#ifdef _MSC_VER
#include <intrin.h>
#endif

#include <cmath>
#include <cstdint>

namespace lbvh {

//! This is the type used for size values.
using size_type = std::size_t;

//! This class is used to associate certain
//! types with a type size.
template<size_type type_size>
struct associated_types final
{};

//! \brief This class is used to describe
//! the amount of work that a task is assigned.
struct work_division final
{
  //! The starting index of the work amount.
  size_type idx;
  //! The maximum divisions of the work.
  size_type max;
};

//! \brief This is a task scheduler that schedules
//! only the current thread for work. It's
//! a placeholder if the user doesn't specifier
//! their own thread pool library.
class single_thread_scheduler final
{
public:
  //! Issues a new task to be performed.
  //! In this class, the task immediately is called in the current thread.
  template<typename task_type, typename... arg_types>
  inline void operator()(task_type task, arg_types... args) noexcept
  {
    task(work_division{ 0, 1 }, args...);
  }
  //! Indicates the maximum number of threads
  //! to be invoked at a time.
  inline size_type max_threads() const noexcept { return 1; }
};

#ifndef LBVH_NO_THREADS

//! \brief This class is used to schedule
//! tasks to be run on several threads. It
//! is not a full-fledged thread pool, as would
//! be desired by real time rendering projects.
//!
//! However, it does improve the performance of
//! the library substantially if it is used.
//!
//! The only reason this thread scheduler is considered
//! to be "naive" is because it creates and destroys threads
//! at each call. Ideally, the threads would be in a thread pool.
class naive_thread_scheduler final
{
  //! The maximum number of threads to run.
  size_type max_thread_count;

public:
  //! Constructs a new fake task scheduler.
  //! \param max_threads_ The maximum number of threads to run.
  naive_thread_scheduler(
    size_type max_threads_ = std::thread::hardware_concurrency())
    : max_thread_count(max_threads_ ? max_threads_ : 1)
  {}
  //! Schedules a new task to be completed.
  //! \tparam task_type The type of the task functor.
  //! \tparam arg_types The arguments to pass to the thread.
  template<typename task_type, typename... arg_types>
  void operator()(task_type task, arg_types... args)
  {
    std::vector<std::thread> threads;

    for (size_type i = 0; i < max_thread_count - 1; i++) {
      work_division div{ i, max_thread_count };

      threads.emplace_back(task, div, args...);
    }

    task(work_division{ max_thread_count - 1, max_thread_count }, args...);

    for (auto& th : threads) {
      th.join();
    }
  }
  //! Indicates to the library the maximum number of threads
  //! that may be invoked at a time. This is useful for determining
  //! how much memory to allocate for some functions.
  //!
  //! \return The max number of threads that may be invoked at a time.
  inline size_type max_threads() const noexcept { return max_thread_count; }
};

//! A type definition that uses the
//! naive thread scheduler.
using default_scheduler = naive_thread_scheduler;

#else // LBVH_NO_THREADS

//! A type definition that uses the
//! single thread scheduler.
using default_scheduler = single_thread_scheduler;

#endif // LBVH_NO_THREADS

//! \brief Calculates the highest bit for an integer type.
//!
//! \return An integer with the highest bit set to one.
template<typename int_type>
inline constexpr int_type
highest_bit() noexcept
{
  return int_type(1) << ((sizeof(int_type) * 8) - 1);
}

//! Represents a single 2D vector.
//!
//! \tparam scalar_type The type of the vector components.
template<typename scalar_type>
struct vec2 final
{
  //! The X component of the vector.
  scalar_type x;
  //! The Y component of the vector.
  scalar_type y;
};

//! Represents a single 3D vector.
//!
//! \tparam scalar_type The type of the vector components.
template<typename scalar_type>
struct vec3 final
{
  //! The X component of the vector.
  scalar_type x;
  //! The Y component of the vector.
  scalar_type y;
  //! The Z component of the vector.
  scalar_type z;
};

//! Represents a single axis-aligned bounding box.
//! \tparam scalar_type The scalar type used for vector values.
template<typename scalar_type>
struct aabb final
{
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
template<typename scalar_type>
struct alignas(sizeof(scalar_type) * 8) node final
{
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
  inline constexpr index_type left_leaf_index() const noexcept
  {
    return left & (highest_bit<index_type>() - 1);
  }
  //! Accesses the right index as a leaf index.
  inline constexpr index_type right_leaf_index() const noexcept
  {
    return right & (highest_bit<index_type>() - 1);
  }
  //! Indicates if the left index points to a leaf.
  inline constexpr bool left_is_leaf() const noexcept
  {
    return left & highest_bit<index_type>();
  }
  //! Indicates if the right index points to a leaf.
  inline constexpr bool right_is_leaf() const noexcept
  {
    return right & highest_bit<index_type>();
  }
};

//! This is the structure for the LBVH.
//! \tparam float_type The floating point type used for box vectors.
template<typename float_type>
class bvh final
{
public:
  //! A type definition for BVH nodes.
  using node_type = node<float_type>;
  //! A type definition for a BVH node vector.
  using node_vec = std::vector<node_type>;
  //! Constructs a BVH from prebuilt internal nodes.
  bvh(node_vec&& nodes_)
    : nodes(std::move(nodes_))
  {}
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
  inline const node_type& at(size_type index) const { return nodes.at(index); }
  //! Accesses a reference to a node within the BVH.
  //! This function does not perform bounds checking.
  //!
  //! \param index The index of the node to access.
  //!
  //! \returns A const-reference to the box type.
  inline const node_type& operator[](size_type index) const noexcept
  {
    return nodes[index];
  }

private:
  //! The internal nodes of the BVH.
  node_vec nodes;
};

//! \brief This class is used for the constructing of BVHs.
//! It is meant to be the first class declared by anyone using the library.
//! It takes a set of primitives, as well as a function object to calculate
//! their bounding boxes, and generates an LBVH from them. The only function
//! required to be implemented is the function object that converts primitives
//! to bounding boxes.
//!
//! \tparam scalar_type The scalar type to use for the box vectors.
//!
//! \tparam task_scheduler The scheduler type to use for construction tasks.
//! By default, this is the type of scheduler provided by the library.
template<typename scalar_type, typename task_scheduler = default_scheduler>
class builder final
{
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
    : scheduler(scheduler_)
  {}
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
  template<typename primitive, typename aabb_converter>
  bvh_type operator()(const primitive* primitives,
                      size_type count,
                      const aabb_converter& converter);

protected:
  //! Fits BVH nodes with their appropriate boxes.
  template<typename primitive, typename aabb_converter>
  void fit_boxes(node_vec& nodes,
                 const primitive* primitives,
                 const aabb_converter& converter);
};

//! \brief This structure contains basic information
//! regarding a ray intersection with a BVH. It stores the index of the
//! primitive that was intersected as well as intersection info given by the
//! primitive type.
template<typename scalar_type, typename intersection_info>
struct intersection_container final
{
  //! A type definition for an index, used for tracking the intersected
  //! primitive.
  using index = typename associated_types<sizeof(scalar_type)>::uint_type;
  /// The index of the primitive that was hit.
  index primitive = std::numeric_limits<index>::max();
  /// Additional information regarding the intersection.
  intersection_info info;

  intersection_container() = default;

  intersection_container(index prim, intersection_info&& info_)
    : primitive(prim)
    , info(std::move(info_))
  {}

  /// Indicates whether or not this is a valid intersection.
  constexpr operator bool() const noexcept
  {
    return primitive != std::numeric_limits<index>::max();
  }
};

//! \brief Contains data for a single ray.
//!
//! \tparam scalar_type The type used for the vector components.
template<typename scalar_type>
struct ray final
{
  //! A type definition for vectors used by the ray class.
  using vec_type = vec3<scalar_type>;
  //! The starting position of the ray.
  vec_type pos;
  //! The direction at which the ray is pointing at.
  //! Usually, this is not normalized.
  vec_type dir;
  //! The minimum distance that intersections can be accepted.
  scalar_type tmin = 0;
  //! The maximum distance that intersections can be accepted.
  scalar_type tmax = std::numeric_limits<scalar_type>::infinity();
};

//! \brief Represents a single ray with
//! precomputed acceleration data.
//!
//! \tparam scalar_type The scalar type of the vector components.
template<typename scalar_type>
struct accel_ray final
{
  //! A type definition for the ray used by this structure.
  using ray_type = ray<scalar_type>;
  //! A type definition for vector types used by this structure.
  using vec3_type = vec3<scalar_type>;
  //! The type to use for octant indices.
  using index_type = typename associated_types<sizeof(scalar_type)>::uint_type;
  //! The original ray from which the acceleration
  //! data was computed.
  ray_type r;
  //! The reciprocal direction vector.
  vec3_type rcp_dir;
  //! The inverse position vector.
  vec3_type inv_pos;
  //! The indices for the octant that the ray falls into.
  index_type octants[3];
  //! The inverse octant indices.
  index_type inv_octants[3];
};

//! \brief A packet of 3D vectors,
//! arranged for decent alignment.
//!
//! \tparam scalar_type The type of the vector components.
//!
//! \tparam dimensions The number of dimensions in the vector.
//!
//! \tparam count The number of elements per dimension.
template<typename scalar_type, size_type dimensions, size_type count>
struct vec_packet final
{
  //! Indicates the total number of scalars in this vector packet.
  static inline constexpr size_type value_count() noexcept
  {
    return dimensions * count;
  }
  //! Indicates the number of scalars per dimension.
  static inline constexpr size_type size() noexcept { return count; }
  //! Accesses a pointer to a dimension within the vector packet.
  //!
  //! \param d The index of the dimension to get the values of.
  //!
  //! \return A pointer to the start of the dimension values.
  inline auto* operator[](size_type d) noexcept { return &values[d * count]; }
  //! Accesses a pointer to a dimension within the vector packet.
  //!
  //! \param d The index of the dimension to get the values of.
  //!
  //! \return A pointer to the start of the dimension values.
  inline const auto* operator[](size_type d) const noexcept
  {
    return &values[d * count];
  }
  //! The values of the vector packet.
  //! Dimensions are non-interleaved in this array.
  scalar_type values[value_count()]{};
};

//! This class is used to store a packet of rays.
//! A packet of rays can be traversed faster than
//! one ray at a time because it leads to better
//! caching, auto-vectorization, and better coherency.
//!
//! \tparam scalar_type The type to use for vector components.
//!
//! \tparam count The maximum number of rays to put into the packet.
template<typename scalar_type, size_type count>
struct ray_packet final
{
  //! A type definition for ray indices.
  using index_type = typename associated_types<sizeof(scalar_type)>::uint_type;
  //! The position vectors of the rays.
  vec_packet<scalar_type, 3, count> pos;
  //! The direction vectors of the rays.
  vec_packet<scalar_type, 3, count> dir;
  //! The reciprocal direction vectors.
  vec_packet<scalar_type, 3, count> rcp_dir;
  //! These are the indices that each ray corresponds to.
  //! Since a ray packet may be sorted multiple times
  //! throughout a BVH traversal, tracking their original
  //! indices allows them to be reordered after a BVH
  //! node is exited.
  index_type indices[count];
};

//! \brief This class is used for traversing a BVH.
//!
//! \tparam scalar_type The floating point type to use for vector components.
//!
//! \tparam primitive_type The type of the primitive being checked for
//! intersection.
//!
//! \tparam intersection_type The type used for indicating intersections.
template<typename scalar_type,
         typename primitive_type,
         typename intersection_info>
class traverser final
{
  //! A reference to the BVH being traversed.
  const bvh<scalar_type>& bvh_;
  //! The primitives to check for intersection.
  const primitive_type* primitives;

public:
  using intersection = intersection_container<scalar_type, intersection_info>;

  //! A type definition for a ray.
  using ray_type = ray<scalar_type>;
  //! Constructs a new traverser instance.
  //! \param b The BVH to be traversed.
  //! \param p The primitives to check for intersection in each box.
  constexpr traverser(const bvh<scalar_type>& b,
                      const primitive_type* p) noexcept
    : bvh_(b)
    , primitives(p)
  {}
  //! \brief Traverses the BVH, returning the closest intersection that was
  //! made.
  //!
  //! \tparam intersector_type Defined by the caller as a function object that
  //! takes a primitive and a ray and returns an instance of @ref
  //! intersection_type that indicates whether or not a hit was made.
  template<typename intersector_type>
  intersection operator()(const ray_type& ray,
                          const intersector_type& intersector) const noexcept;
};

//! \brief Contains the associated types for 32-bit sizes.
template<>
struct associated_types<4> final
{
  //! The unsigned integer type to use for 32-bit types.
  using uint_type = std::uint32_t;
  //! The signed integer type to use for 32-bit types.
  using int_type = std::int32_t;
  //! The floating point type to use for 32-bit types.
  using float_type = float;
};

//! \brief Contains the associated types for 64-bit sizes.
template<>
struct associated_types<8> final
{
  //! The unsigned integer type to use for 64-bit types.
  using uint_type = std::uint64_t;
  //! The signed integer type to use for 64-bit types.
  using int_type = std::int64_t;
  //! The floating point type to use for 64-bit types.
  using float_type = double;
};

//! \brief This namespace contains various math routines
//! for scalar and vector types. It may be useful to the
//! user when writing intersection functions or primitive
//! to box conversions.
namespace math {

//! \brief Calculates the minimum of two scalar values.
template<typename scalar_type>
inline constexpr scalar_type
min(scalar_type a, scalar_type b) noexcept
{
  return (a < b) ? a : b;
}

//! \brief Calculates the maximum of two scalar values.
template<typename scalar_type>
inline constexpr scalar_type
max(scalar_type a, scalar_type b) noexcept
{
  return (a > b) ? a : b;
}

//! \brief Calculates the Hadamard quotient of two vectors.
//!
//! \tparam scalar_type The scalar type of the vector components.
//!
//! \return The Hadamard quotient of the two vectors.
template<typename scalar_type>
auto
hadamard_div(const vec3<scalar_type>& a, const vec3<scalar_type>& b) noexcept
{
  return vec3<scalar_type>{ a.x / b.x, a.y / b.y, a.z / b.z };
}

//! \brief Calculates the Hadamard product of two vectors.
//!
//! \tparam scalar_type The scalar type of the vector components.
//!
//! \return The Hadamard product of the two vectors.
template<typename scalar_type>
auto
hadamard_mul(const vec3<scalar_type>& a, const vec3<scalar_type>& b) noexcept
{
  return vec3<scalar_type>{ a.x * b.x, a.y * b.y, a.z * b.z };
}

//! \brief Calculates the cross product between two 3D vectors.
//!
//! \return The cross product of @p a and @p b.
template<typename scalar_type>
inline constexpr vec3<scalar_type>
cross(const vec3<scalar_type>& a, const vec3<scalar_type>& b) noexcept
{
  return vec3<scalar_type>{ (a.y * b.z) - (a.z * b.y),
                            (a.z * b.x) - (a.x * b.z),
                            (a.x * b.y) - (a.y * b.x) };
}

//! \brief Calculates the dot product of two 3D vectors.
//!
//! \return The dot product of @p a and @p b.
template<typename scalar_type>
inline constexpr scalar_type
dot(const vec3<scalar_type>& a, const vec3<scalar_type>& b) noexcept
{
  scalar_type out = 0;
  out += a.x * b.x;
  out += a.y * b.y;
  out += a.z * b.z;
  return out;
}

//! \brief Calculates the minimum between two vectors.
//! This is used frequently in bounding box calculations.
//!
//! \tparam scalar_type The scalar type of the vector components.
//!
//! \return A vector containing the minimum components between @p a and @p b.
template<typename scalar_type>
auto
min(const vec3<scalar_type>& a, const vec3<scalar_type>& b) noexcept
{
  return vec3<scalar_type>{ std::min(a.x, b.x),
                            std::min(a.y, b.y),
                            std::min(a.z, b.z) };
}

//! \brief Calculates the maximum between two vectors.
//! This is used frequently in bounding box calculations.
//!
//! \tparam scalar_type The scalar type of the vector components.
//!
//! \return A vector containing the maximum components between @p a and @p b.
template<typename scalar_type>
auto
max(const vec3<scalar_type>& a, const vec3<scalar_type>& b) noexcept
{
  return vec3<scalar_type>{ std::max(a.x, b.x),
                            std::max(a.y, b.y),
                            std::max(a.z, b.z) };
}

//! \brief Calculates a vector with reciprocal values of another vector.
//! This is used for ray-box intersection acceleration.
//!
//! \tparam scalar_type The type used for vector components.
//!
//! \return A vector with reciprocal values of @p in.
template<typename scalar_type>
auto
reciprocal(const vec3<scalar_type>& in) noexcept
{
  return vec3<scalar_type>{ 1 / in.x, 1 / in.y, 1 / in.z };
}

//! \brief Calculates the sum of two vectors.
//!
//! \tparam scalar_type The type of the vector components.
//!
//! \return The sum of @p a and @p b, as a vector.
template<typename scalar_type>
auto
operator+(const vec3<scalar_type>& a, const vec3<scalar_type>& b) noexcept
{
  return vec3<scalar_type>{ a.x + b.x, a.y + b.y, a.z + b.z };
}

//! \brief Calculates the difference between two vectors.
//!
//! \tparam scalar_type The type of the vector components.
//!
//! \return The difference between @p a and @p b, as a vector.
template<typename scalar_type>
auto
operator-(const vec3<scalar_type>& a, const vec3<scalar_type>& b) noexcept
{
  return vec3<scalar_type>{ a.x - b.x, a.y - b.y, a.z - b.z };
}

//! \brief Negates a vector.
//!
//! \return The negated result of @p in.
template<typename scalar_type>
inline auto
operator-(const vec3<scalar_type>& in) noexcept
{
  return vec3<scalar_type>{ -in.x, -in.y, -in.z };
}

//! \brief Calculates the product between a vector and a scalar value.
//!
//! \tparam scalar_type The scalar type of the vector.
//!
//! \return The product between @p a and @p b.
template<typename scalar_type>
auto
operator*(const vec3<scalar_type>& a, scalar_type b) noexcept
{
  return vec3<scalar_type>{ a.x * b, a.y * b, a.z * b };
}

//! \brief Normalizes a vector.
//! This is useful when dealing with trig functions.
//!
//! \return The normalized result of @p v.
template<typename scalar_type>
vec3<scalar_type>
normalize(const vec3<scalar_type>& v) noexcept
{
  using std::sqrt;

  auto l_inv = scalar_type(1) / sqrt(dot(v, v));

  return v * l_inv;
}

//! \brief Multiplies a 2D vector by a scalar value.
//!
//! \return The product of @p a and @p b.
template<typename scalar_type>
vec2<scalar_type>
operator*(const vec2<scalar_type>& a, scalar_type b) noexcept
{
  return vec2<scalar_type>{ a.x * b, a.y * b };
}

//! \brief Computes the sum of two 2D vectors.
//!
//! \return The sum of @p a and @p b.
template<typename scalar_type>
vec2<scalar_type>
operator+(const vec2<scalar_type>& a, const vec2<scalar_type>& b) noexcept
{
  return vec2<scalar_type>{ a.x + b.x, a.y + b.y };
}

//! \brief This structure implements various vector packet operations.
//! Since these operations require various template parameters, they are
//! best implemented in a structure where type definitions can make the code
//! much more readable.
//!
//! \tparam scalar_type The scalar type of the vector components.
//!
//! \tparam dimensions The number of dimensions the vectors will have.
//!
//! \tparam count The number of elements per dimension for each vector.
template<typename scalar_type, size_type dimensions, size_type count>
struct vec_packet_ops final
{
  //! A type definition for a vector packet.
  using vec_packet_type = vec_packet<scalar_type, dimensions, count>;
  //! The number of values in one vector packet.
  static inline constexpr auto value_count() noexcept
  {
    return dimensions * count;
  }
  //! Computes the Hadamard product of two vectors.
  //!
  //! \return The Hadamard product of @p a and @p b.
  static constexpr auto hadamard_mul(const vec_packet_type& a,
                                     const vec_packet_type& b) noexcept
  {
    vec_packet_type out;
    for (size_type i = 0; i < value_count(); i++) {
      out.values[i] = a.values[i] * b.values[i];
    }
    return out;
  }
  //! Computes the sum of a vector packet and single scalar value.
  //!
  //! \param b This value is added to all components of @p a.
  //!
  //! \return The sum of @p a and @p b.
  static constexpr auto add(const vec_packet_type& a, scalar_type b) noexcept
  {
    vec_packet_type out;
    for (size_type i = 0; i < value_count(); i++) {
      out.values[i] = a.values[i] + b;
    }
    return out;
  }
  //! Subtracts two vectors.
  //!
  //! \return The difference between @p a and @p b.
  static constexpr auto sub(const vec_packet_type& a,
                            const vec_packet_type& b) noexcept
  {
    vec_packet_type out;
    for (size_type i = 0; i < value_count(); i++) {
      out.values[i] = a.values[i] - b.values[i];
    }
    return out;
  }
  //! Multiplies a vector packet by a scalar value.
  //!
  //! \return The product of @p a and @p b.
  static constexpr auto mul(const vec_packet_type& a, scalar_type b) noexcept
  {
    vec_packet_type out;
    for (size_type i = 0; i < value_count(); i++) {
      out.values[i] = a.values[i] * b;
    }
    return out;
  }
};

//! Adds a single scalar value to all components of @p a.
//!
//! \return The sume of @p a and @p b.
template<typename scalar_type, size_type dimensions, size_type count>
constexpr auto
operator+(const vec_packet<scalar_type, dimensions, count>& a,
          scalar_type b) noexcept
{
  return vec_packet_ops<scalar_type, dimensions, count>::add(a, b);
}

//! Subtracts two vector packets.
//!
//! \return The difference of @p a and @p b.
template<typename scalar_type, size_type dimensions, size_type count>
constexpr auto
operator-(const vec_packet<scalar_type, dimensions, count>& a,
          const vec_packet<scalar_type, dimensions, count>& b) noexcept
{
  return vec_packet_ops<scalar_type, dimensions, count>::sub(a, b);
}

//! Multiplies a vector packet by a single scalar value.
//!
//! \return The product of @p a and @p b, as a vector packet.
template<typename scalar_type, size_type dimensions, size_type count>
constexpr auto
operator*(const vec_packet<scalar_type, dimensions, count>& a,
          scalar_type b) noexcept
{
  return vec_packet_ops<scalar_type, dimensions, count>::mul(a, b);
}

//! Multiplies a 3D vector packet by a single 3D vector.
//!
//! \tparam scalar_type The scalar type of the vector components.
//!
//! \tparam count The number of elements per dimension in the vector packet.
//!
//! \return The product of @p a and @p b.
template<typename scalar_type, size_type count>
constexpr auto
hadamard_mul(const vec_packet<scalar_type, 3, count>& a,
             const vec3<scalar_type>& b) noexcept
{
  vec_packet<scalar_type, 3, count> out;

  for (size_type i = 0; i < count; i++) {
    out[0][i] = a[0][i] * b.x;
    out[1][i] = a[1][i] * b.y;
    out[2][i] = a[2][i] * b.z;
  }

  return out;
}

//! \brief Subtracts a single 3D vector from a 3D vector packet.
//!
//! \tparam scalar_type The type used by the vector components.
//!
//! \tparam count The number of elements in one dimension.
//!
//! \return The difference between @p a and @p b.
template<typename scalar_type, size_type count>
constexpr auto
operator-(const vec_packet<scalar_type, 3, count>& a,
          const vec3<scalar_type>& b) noexcept
{
  vec_packet<scalar_type, 3, count> out;

  for (size_type i = 0; i < count; i++) {
    out[0][i] = a[0][i] - b.x;
    out[1][i] = a[1][i] - b.y;
    out[2][i] = a[2][i] - b.z;
  }

  return out;
}

} // namespace math

//! \brief This namespaces contains implementation details
//! of the library. It shouldn't be used by the end-user.
namespace detail {

using namespace lbvh::math;

//! \brief Counts leading zeroes of a 32-bit integer.
inline auto
clz(std::uint32_t n) noexcept
{
#ifdef _MSC_VER
  return __lzcnt(n);
#else
  return __builtin_clz(n);
#endif
}

//! \brief Counts leading zeroes of a 64-bit integer.
inline auto
clz(std::uint64_t n) noexcept
{
#ifdef _MSC_VER
  return __lzcnt64(n);
#else
  return __builtin_clzll(n);
#endif
}

//! \brief Gets the sign of a number, as an integer.
//!
//! \return The sign of @p n. If @p n is negative,
//! then negative one is returned. If @p n is greater
//! than or equal to positive zero, then positive one
//! is returned.
template<typename scalar_type>
inline constexpr scalar_type
sign(scalar_type n) noexcept
{
  return (n < 0) ? -1 : 1;
}

//! Divides and rounds up to the nearest integer.
//!
//! \tparam int_type The integer type to divide with.
//!
//! \return The quotient between @p n and @ d, rounded up.
template<typename int_type>
inline constexpr int_type
ceil_div(int_type n, int_type d)
{
  return (n + (d - 1)) / d;
}

//! Used for iterating a range of numbers in a for loop.
struct loop_range final
{
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
  loop_range(const work_division& div, size_type array_size) noexcept
  {
    auto chunk_size = array_size / div.max;

    begin = chunk_size * div.idx;

    if ((div.idx + 1) == div.max) {
      end = array_size;
    } else {
      end = begin + chunk_size;
    }
  }
};

//! Constructs an accelerated ray structure.
//!
//! \param r The ray from which to build the structure from.
template<typename scalar_type>
constexpr auto
make_accel_ray(const ray<scalar_type>& r) noexcept
{
  using index_type = typename accel_ray<scalar_type>::index_type;

  auto rcp_dir = reciprocal(r.dir);

  index_type octants[3]{ (r.dir.x > 0) ? index_type(0) : index_type(3),
                         (r.dir.y > 0) ? index_type(1) : index_type(4),
                         (r.dir.z > 0) ? index_type(2) : index_type(5) };

  return accel_ray<scalar_type>{
    r,
    rcp_dir,
    hadamard_mul(-r.pos, rcp_dir),
    { octants[0], octants[1], octants[2] },
    { 3 - octants[0], 5 - octants[1], 7 - octants[2] }
  };
}

//! Represents a ray intersection with a box.
//!
//! \tparam scalar_type The scalar type of the intersection distances.
template<typename scalar_type>
struct box_intersection final
{
  //! The minimum distance to intersection.
  scalar_type tmin = std::numeric_limits<scalar_type>::infinity();
  //! The maximum distance to intersection.
  scalar_type tmax = 0;
  //! Indicates if this is a valid intersection.
  inline operator bool() const noexcept
  {
    return (tmax >= max(scalar_type(0), tmin));
  }
  //! Compares two box intersections by proximity.
  //!
  //! \return True if this box intersection is closer than @p other.
  inline bool operator<(const box_intersection& other) const noexcept
  {
    return tmin < other.tmin;
  }
};

//! Checks for ray intersection with a bounding box.
//!
//! \tparam scalar_type The type used for vector components.
//!
//! \return A box intersection instance, indicating
//! if there was a hit or not.
template<typename scalar_type>
auto
intersect(const aabb<scalar_type>& box,
          const accel_ray<scalar_type>& accel_r) noexcept
{
#ifdef LBVH_ENABLE_SLAB_TEST

  auto tx1 = (box.min.x - accel_r.r.pos.x) * accel_r.rcp_dir.x;
  auto tx2 = (box.max.x - accel_r.r.pos.x) * accel_r.rcp_dir.x;

  auto tmin = min(tx1, tx2);
  auto tmax = max(tx1, tx2);

  auto ty1 = (box.min.y - accel_r.r.pos.y) * accel_r.rcp_dir.y;
  auto ty2 = (box.max.y - accel_r.r.pos.y) * accel_r.rcp_dir.y;

  tmin = max(tmin, min(ty1, ty2));
  tmax = min(tmax, max(ty1, ty2));

  auto tz1 = (box.min.z - accel_r.r.pos.z) * accel_r.rcp_dir.z;
  auto tz2 = (box.max.z - accel_r.r.pos.z) * accel_r.rcp_dir.z;

  tmin = max(tmin, min(tz1, tz2));
  tmax = min(tmax, max(tz1, tz2));

  return box_intersection<scalar_type>{ tmin, tmax };

#else // LBVH_ENABLE_SLAB_TEST

  scalar_type bounds[6]{ box.min.x, box.min.y, box.min.z,
                         box.max.x, box.max.y, box.max.z };

  // t near
  scalar_type tn[3]{
    (bounds[accel_r.octants[0]] * accel_r.rcp_dir.x) + accel_r.inv_pos.x,
    (bounds[accel_r.octants[1]] * accel_r.rcp_dir.y) + accel_r.inv_pos.y,
    (bounds[accel_r.octants[2]] * accel_r.rcp_dir.z) + accel_r.inv_pos.z
  };

  // t far
  scalar_type tf[3]{
    (bounds[accel_r.inv_octants[0]] * accel_r.rcp_dir.x) + accel_r.inv_pos.x,
    (bounds[accel_r.inv_octants[1]] * accel_r.rcp_dir.y) + accel_r.inv_pos.y,
    (bounds[accel_r.inv_octants[2]] * accel_r.rcp_dir.z) + accel_r.inv_pos.z
  };

  return box_intersection<scalar_type>{
    max(max(tn[0], tn[1]), tn[2]),
    min(min(tf[0], tf[1]), tf[2]),
  };

#endif // LBVH_ENABLE_SLAB_TEST
}

//! \brief This class represents a space filling curve.
//!
//! \tparam code_type The type used for units of the curve.
template<typename code_type>
class space_filling_curve final
{
public:
  //! Represents a single entry within the curve table.
  struct entry final
  {
    //! Let's make the code and the primitive index the same size
    //! by using this type definition.
    using index_type = typename associated_types<sizeof(code_type)>::uint_type;
    //! The code at this point along the curve.
    code_type code;
    //! The index to the primitive associated with this point.
    index_type primitive;
  };
  //! A type definition for a vector of entries.
  using entry_vec = std::vector<entry>;
  //! Constructs a space filling curve.
  //! \param entries_ The entries to assign the curve.
  space_filling_curve(entry_vec&& entries_) noexcept
    : entries(std::move(entries_))
  {}
  //! Constructs a curve via move semantics.
  space_filling_curve(space_filling_curve&& other) noexcept
    : entries(std::move(other.entries))
  {}
  //! Sorts the space filling curve based on the code of each entry.
  void sort()
  {
    auto cmp = [](const entry& a, const entry& b) { return a.code < b.code; };
#if (__cplusplus >= 201703L) && !(defined LBVH_NO_THREADS)
    std::sort(std::execution::par_unseq, entries.begin(), entries.end(), cmp);
#else
    std::sort(entries.begin(), entries.end(), cmp);
#endif
  }
  //! Indicates the number of entries in the space filling curve.
  inline auto size() const noexcept { return entries.size(); }
  //! Accesses a specific entry within the space filling curve.
  //! \param index The index of the entry to access.
  inline const auto& operator[](size_type index) const noexcept
  {
    return entries[index];
  }

  space_filling_curve(const space_filling_curve&) = delete;
  space_filling_curve& operator=(const space_filling_curve&) = delete;

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
template<
  typename scalar_type,
  typename = std::enable_if_t<std::is_floating_point<scalar_type>::value>>
auto
center_of(const aabb<scalar_type>& box) noexcept
{
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
template<typename scalar_type,
         std::enable_if_t<!std::is_floating_point<scalar_type>::value>>
auto
center_of(const aabb<scalar_type>& box) noexcept
{
  return (box.min + box.max) / 2;
}

//! \brief Gets an empty bounding box.
//! An empty box has the property that any union this
//! box has with another box is equal to the other box.
//!
//! \return An empty box.
template<typename scalar_type>
auto
get_empty_aabb() noexcept
{
  return aabb<scalar_type>{ { std::numeric_limits<scalar_type>::infinity(),
                              std::numeric_limits<scalar_type>::infinity(),
                              std::numeric_limits<scalar_type>::infinity() },
                            { -std::numeric_limits<scalar_type>::infinity(),
                              -std::numeric_limits<scalar_type>::infinity(),
                              -std::numeric_limits<scalar_type>::infinity() } };
}

//! \brief Calculates the union of two bounding boxes.
//! The result is a bounding box that fits both other bounding boxes.
//!
//! \tparam scalar_type The type of the bounding box vector components.
//!
//! \return A bounding box that fits both boxes passed as parameters.
template<typename scalar_type>
auto
union_of(const aabb<scalar_type>& a, const aabb<scalar_type>& b) noexcept
{
  return aabb<scalar_type>{ min(a.min, b.min), max(a.max, b.max) };
}

//! \brief Calculates the union of a bounding box and a 3D vector.
//!
//! \return A bounding box fitting both @p a and @p b.
template<typename scalar_type>
auto
union_of(const aabb<scalar_type>& a, const vec3<scalar_type>& b) noexcept
{
  return aabb<scalar_type>{ min(a.min, b), max(a.max, b) };
}

//! \brief Calculates the size of a bounding box.
//! This is considered the change in value from minimum to maximum points.
//!
//! \tparam scalar_type The type of the bounding box vector components.
//!
//! \param box The box to get the size of.
//!
//! \return The size of the given box.
template<typename scalar_type>
auto
size_of(const aabb<scalar_type>& box) noexcept
{
  return box.max - box.min;
}

//! \brief This class is used for calculating the centroid boundaries in the
//! scene.
//!
//! \tparam scalar_type The scalar type of the bounding box to get.
//!
//! \tparam primitive_type The type of primitive in the scene.
//!
//! \tparam aabb_converter Calculates the bounding box of a primitive.
template<typename scalar_type, typename primitive_type, typename aabb_converter>
class centroid_bounds_kernel final
{
public:
  //! A type definition for a type returned by this class.
  using box_type = aabb<scalar_type>;
  //! Constructs a new centroid bounds kernel.
  //!
  //! \param p The primitive of arrays to get the bounds for.
  //!
  //! \param c The number of primitives in the array.
  //!
  //! \param cvt The primitive to bounding box converter.
  //!
  //! \param thb The array of boxes per thread. Each thread
  //! will find a box for a certain portion of the scene. This
  //! array should have as many boxes as there are threads.
  centroid_bounds_kernel(const primitive_type* p,
                         size_type c,
                         const aabb_converter& cvt,
                         box_type* thb)
    : primitives(p)
    , count(c)
    , converter(cvt)
    , thread_boxes(thb)
  {}
  //! Runs the kernel.
  //! Each thread accumulates its own bounding box
  //! for the portion of the scene it was assigned.
  //! When the result is queried, all boxes for each
  //! of the threads are put into union.
  //!
  //! \param div Given by the scheduler to indicate which
  //! portion of the scene this call should get the bounding box of.
  void operator()(const work_division& div) noexcept
  {
    auto range = loop_range(div, count);

    auto box = get_empty_aabb<scalar_type>();

    for (size_type i = range.begin; i < range.end; i++) {
      box = union_of(box, center_of(converter(primitives[i])));
    }

    thread_boxes[div.idx] = box;
  }

private:
  //! The array of primitives to get the bounding box of.
  const primitive_type* primitives;
  //! The number of primitives in the primitive array.
  size_type count;
  //! The primitive to bounding box converter.
  const aabb_converter& converter;
  //! The array of boxes, each box allocated
  //! for a thread.
  box_type* thread_boxes;
};

//! \brief Used to get the domain of Morton coordinates,
//! based on the size of the type being used.
template<size_type type_size>
struct morton_domain final
{};

//! \brief Contains the domain of 32-bit Morton codes.
template<>
struct morton_domain<4> final
{
  //! Gets the domain of 32-bit Morton codes.
  static constexpr size_type value() noexcept { return 1024; }
};

//! \brief Contains the domain of 64-bit Morton codes.
template<>
struct morton_domain<8> final
{
  //! Gets the domain of 64-bit Morton codes.
  static constexpr size_type value() noexcept { return 1048576; }
};

//! \brief This class is used for encoding Morton values.
//! This class is specialized based on the size of a code point.
//!
//! \tparam type_size The type size of a code point.
template<size_type type_size>
class morton_encoder final
{};

//! \brief Used for encoding 32-bit Morton codes.
template<>
class morton_encoder<4> final
{
public:
  //! A type definition for a 32-bit Morton code.
  using code_type = associated_types<4>::uint_type;
  //! Encodes a 3D 32-bit Morton code.
  inline code_type operator()(code_type x, code_type y, code_type z) noexcept
  {
    return (expand(x) << 2) | (expand(y) << 1) | (expand(z) << 0);
  }

protected:
  //! Expands a 10-bit value to 30-bits.
  inline static code_type expand(code_type n) noexcept
  {
    n = (n | (n << 16)) & 0x030000ff;
    n = (n | (n << 8)) & 0x0300f00f;
    n = (n | (n << 4)) & 0x030c30c3;
    n = (n | (n << 2)) & 0x09249249;
    return n;
  }
};

//! \brief Used for encoding 64-bit Morton codes.
template<>
class morton_encoder<8> final
{
public:
  //! A type definition for a 64-bit Morton code.
  using code_type = associated_types<8>::uint_type;
  //! Encodes a 3D 64-bit Morton code.
  inline code_type operator()(code_type x, code_type y, code_type z) noexcept
  {
    return (expand(x) << 2) | (expand(y) << 1) | (expand(z) << 0);
  }

protected:
  //! Expands a 20-bit value to 60-bits.
  inline static code_type expand(code_type n) noexcept
  {
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
template<typename scalar_type, typename primitive_type>
class morton_curve_kernel final
{
public:
  //! A type definition for a code value.
  using code_type = typename associated_types<sizeof(scalar_type)>::uint_type;
  //! A type definition for an entry in the space filling curve.
  using entry = typename space_filling_curve<code_type>::entry;
  //! Constructs a new Morton curve kernel.
  //! \param p The primitive array to generate the values from.
  //! \param e The entry array to receive the values.
  //! \param c The number of primitives in the array.
  constexpr morton_curve_kernel(const primitive_type* p,
                                entry* e,
                                size_type c) noexcept
    : primitives(p)
    , entries(e)
    , count(c)
  {}
  //! Calculates the Morton codes of a certain subset of the scene.
  //! The amount of work that's done depends on the work division.
  //!
  //! \param div Passed by the scheduler to indicate the amount of work to do.
  //!
  //! \param centroid_bounds The bounding box for all centroids.
  //!
  //! \param converter The primitive to bounding box converter.
  template<typename aabb_converter>
  void operator()(const work_division& div,
                  const aabb<scalar_type>& centroid_bounds,
                  aabb_converter converter)
  {
    using entry_index_type = typename entry::index_type;

    //! This is the maximum number of primitives to
    //! work on per loop iteration.
    constexpr size_type max_batch_size = 16;

    auto mdomain = code_type(morton_domain<sizeof(scalar_type)>::value());

    auto cbounds_size = size_of(centroid_bounds);

    // The scale at which the centroids map to Morton space.
    vec3<scalar_type> scale{
      mdomain / (cbounds_size.x + scalar_type(1)),
      mdomain / (cbounds_size.y + scalar_type(1)),
      mdomain / (cbounds_size.z + scalar_type(1)),
    };

    morton_encoder<sizeof(code_type)> encoder;

    auto range = loop_range(div, count);

    vec_packet<scalar_type, 3, max_batch_size> center_packet;

    for (size_type i = range.begin; i < range.end; i += max_batch_size) {
      auto batch_size = min(max_batch_size, range.end - i);

      for (size_type j = 0; j < batch_size; j++) {
        auto center = center_of(converter(primitives[i + j]));
        center_packet[0][j] = center.x;
        center_packet[1][j] = center.y;
        center_packet[2][j] = center.z;
      }

      // Scale center points to Morton space [0, 1024)

      center_packet = hadamard_mul(center_packet - centroid_bounds.min, scale);

      // Export Morton codes

      for (size_type j = 0; j < batch_size; j++) {
        auto x_code = code_type(center_packet[0][j]);
        auto y_code = code_type(center_packet[1][j]);
        auto z_code = code_type(center_packet[2][j]);

        auto code = encoder(x_code, y_code, z_code);

        entries[i + j] = entry{ code, entry_index_type(i + j) };
      }
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
template<typename scalar_type, typename task_scheduler>
class morton_curve_builder final
{
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
    : scheduler(scheduler_)
  {}
  //! Converts a set of primitives into a space filling curve.
  //!
  //! \param primitives The array of primitives to convert.
  //!
  //! \param count The number of primitives in the primitive array.
  //!
  //! \param converter The primitive to bounding box converter.
  //!
  //! \return A space filling curve with Morton codes.
  template<typename primitive, typename aabb_converter>
  curve_type operator()(const primitive* primitives,
                        size_type count,
                        const aabb_converter& converter)
  {
    using entry_vec = typename curve_type::entry_vec;

    using box_type = aabb<scalar_type>;

    using centroid_bounds_kernel_type =
      centroid_bounds_kernel<scalar_type, primitive, aabb_converter>;

    std::vector<box_type> thread_boxes(scheduler.max_threads());

    centroid_bounds_kernel_type scene_bounds_kern(
      primitives, count, converter, thread_boxes.data());

    scheduler(scene_bounds_kern);

    auto centroid_bounds = get_empty_aabb<scalar_type>();

    for (const auto& th_box : thread_boxes) {
      centroid_bounds = union_of(centroid_bounds, th_box);
    }

    entry_vec entries(count);

    morton_curve_kernel<scalar_type, primitive> curve_kernel(
      primitives, entries.data(), count);

    scheduler(curve_kernel, centroid_bounds, converter);

    return curve_type(std::move(entries));
  }
};

//! \brief Represents a division of an internal LBVH node.
struct node_division final
{
  //! The first index of the division.
  size_type index_a;
  //! The second index of the division.
  size_type index_b;
  //! The index at which the division occurs.
  size_type split;
  //! Accesses the minimum index of the division.
  inline constexpr auto min() const noexcept
  {
    return (index_a < index_b) ? index_a : index_b;
  }
  //! Accesses the maximum index of the division.
  inline constexpr auto max() const noexcept
  {
    return (index_b > index_a) ? index_b : index_a;
  }
};

//! This function divides an internal node into two ranges.
//!
//! \tparam code_type The type used for codes in the space filling curve.
//!
//! \param table The space filling curve, used to determine the indices of the
//! split.
//!
//! \param node_index The index of the node being divided.
//!
//! \return A division structure instance, which may be used to assign sub
//! nodes.
template<typename code_type>
node_division
divide_node(const space_filling_curve<code_type>& table,
            size_type node_index) noexcept
{
  // Used as the return value of the delta operator.
  using delta_type = typename associated_types<sizeof(code_type)>::int_type;

  // The type used for offsets.
  using offset_type = typename associated_types<sizeof(code_type)>::int_type;

  // Calculates upper bits, or returns -1
  // if 'k' is out of bounds. Appears in the
  // LBVH paper as 'δ' (lower case delta)
  auto calc_delta = [&table](offset_type j, offset_type k) -> delta_type {
    if ((k < 0) || (k >= offset_type(table.size()))) {
      // Suggested by the paper for out of bounds 'k'
      return -1;
    }

    auto l_code = table[size_type(j)].code;
    auto r_code = table[size_type(k)].code;

    using clz_input_type =
      typename associated_types<sizeof(code_type)>::uint_type;

    if (l_code == r_code) {
      // Suggested by the paper in
      // case the code are equal.
      return clz(clz_input_type(j ^ k)) + delta_type(sizeof(code_type) * 8);
    } else {
      return clz(clz_input_type(l_code ^ r_code));
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

  auto gamma = i + (s * d) + min(d, delta_type(0));

  return node_division{ size_type(i), size_type(j), size_type(gamma) };
}

//! \brief Used for building the BVH nodes.
//! Can be called by the scheduler from many threads.
//!
//! \tparam code_type The type of code contained by the space filling curve.
//!
//! \tparam scalar_type The scalar type used by the node boxes.
template<typename code_type, typename scalar_type>
class builder_kernel final
{
public:
  //! A type definition for a space filling curve.
  using curve_type = space_filling_curve<code_type>;
  //! A type definition for a node type.
  using node_type = node<scalar_type>;
  //! Constructs a new builder kernel.
  //! \param c The curve containing the codes to build the nodes with.
  //! \param n The allocated node array to put the node data into.
  constexpr builder_kernel(const curve_type& c, node_type* n) noexcept
    : curve(c)
    , nodes(n)
  {}
  //! Calls the kernel to build a certain portion of the BVH nodes.
  //! \param div The division of work this function call is responsible for.
  void operator()(const work_division& div) noexcept
  {
    using index_type = typename node_type::index_type;

    auto range = loop_range(div, curve.size() - 1);

    for (auto i = range.begin; i < range.end; i++) {
      auto node_div = divide_node(curve, i);

      auto l_is_leaf = (node_div.min() == (node_div.split + 0));
      auto r_is_leaf = (node_div.max() == (node_div.split + 1));

      auto l_mask = l_is_leaf ? highest_bit<index_type>() : 0;
      auto r_mask = r_is_leaf ? highest_bit<index_type>() : 0;

      nodes[i].left = index_type((node_div.split + 0) | l_mask);
      nodes[i].right = index_type((node_div.split + 1) | r_mask);
    }
  }

private:
  //! This is the space filling curve used to determine
  //! the range and split of each node that this kernel will build.
  const curve_type& curve;
  //! A pointer to the node array being constructed.
  node_type* nodes;
};

//! Used for traversing the BVH.
//!
//! \tparam scalar_type The floating point type to use in the traversal.
//!
//! \tparam max The maximum number of entries to allocate on the stack.
template<typename scalar_type, size_type max>
class traversal_stack final
{
public:
  //! The type to be used for a node index.
  using node_index_type =
    typename associated_types<sizeof(scalar_type)>::uint_type;
  //! Contains information regarding a ndoe
  //! to be traversed.
  struct entry final
  {
    //! The index of the node to traverse.
    node_index_type node_index;
    //! The minimum scale at which the ray
    //! hits this node.
    scalar_type tmin;
  };
  //! Indicates the number of entries remaining
  //! in the stack.
  inline size_type remaining() const noexcept { return pos; }
  //! Removes an entry from the stack.
  auto pop() noexcept
  {
    if (!pos) {
      return entry{ 0, std::numeric_limits<scalar_type>::infinity() };
    }
    return entries[--pos];
  }
  //! Pushes an item to the stack.
  //! \param i The index of the node.
  //! \param t The scale at which the ray intersects this node.
  void push(size_type i, scalar_type t) noexcept
  {
    if (pos < max) {
      entries[pos++] = entry{ node_index_type(i), t };
    }
  }

private:
  //! The position of the "stack pointer."
  size_type pos = 0;
  //! The array of entries to be filled.
  entry entries[max];
};

} // namespace detail

template<typename scalar_type, typename task_scheduler>
template<typename primitive, typename aabb_converter>
auto
builder<scalar_type, task_scheduler>::operator()(
  const primitive* primitives,
  size_type count,
  const aabb_converter& converter) -> bvh_type
{
  using curve_builder_type =
    detail::morton_curve_builder<scalar_type, task_scheduler>;

  using code_type = typename curve_builder_type::code_type;

  curve_builder_type curve_builder(scheduler);

  auto curve = curve_builder(primitives, count, converter);

  curve.sort();

  std::vector<node_type> node_vec(curve.size() - 1);

  detail::builder_kernel<code_type, scalar_type> builder_kern(curve,
                                                              node_vec.data());

  scheduler(builder_kern);

  fit_boxes(node_vec, primitives, converter);

  return bvh_type(std::move(node_vec));
}

template<typename scalar_type, typename task_scheduler>
template<typename primitive, typename aabb_converter>
void
builder<scalar_type, task_scheduler>::fit_boxes(node_vec& nodes,
                                                const primitive* primitives,
                                                const aabb_converter& converter)
{
  std::vector<size_type> indices;

  indices.reserve(nodes.size());

  indices.push_back(0);

  for (size_type i = 0; i < indices.size(); i++) {
    auto j = indices[i];

    if (!nodes[j].left_is_leaf()) {
      indices.push_back(nodes[j].left);
    }

    if (!nodes[j].right_is_leaf()) {
      indices.push_back(nodes[j].right);
    }
  }

  for (size_type i = indices.size(); i > 0; i--) {
    auto j = indices[i - 1];

    auto& node = nodes[j];

    if (node.left_is_leaf()) {
      node.box = converter(primitives[node.left_leaf_index()]);
    } else {
      node.box = nodes[node.left].box;
    }

    if (node.right_is_leaf()) {
      node.box = detail::union_of(
        node.box, converter(primitives[node.right_leaf_index()]));
    } else {
      node.box = detail::union_of(node.box, nodes[node.right].box);
    }
  }
}

template<typename scalar_type,
         typename primitive_type,
         typename intersection_type>
template<typename intersector_type>
auto
traverser<scalar_type, primitive_type, intersection_type>::operator()(
  const ray_type& ray,
  const intersector_type& intersector) const noexcept -> intersection
{
  using box_intersection_type = detail::box_intersection<scalar_type>;

  detail::traversal_stack<scalar_type, 128> stack;

  stack.push(0, std::numeric_limits<scalar_type>::infinity());

  auto accel_r = detail::make_accel_ray(ray);

  auto intersect_primitive =
    [](const auto& intersector_, const auto* p, auto index, const auto& r) {
      return intersection(index, intersector_(p[index], r));
    };

  intersection closest;

  while (stack.remaining()) {
    auto entry = stack.pop();

    if (closest.info < entry.tmin) {
      // We've already got a closer intersection than
      // what can be found at this node, we can skip this.
      continue;
    }

    const auto& node = bvh_[entry.node_index];

    box_intersection_type left_box_isect;

    if (node.left_is_leaf()) {
      auto left_isect = intersect_primitive(
        intersector, primitives, node.left_leaf_index(), ray);
      if (left_isect.info < closest.info)
        closest = left_isect;
    } else {
      left_box_isect = detail::intersect(bvh_[node.left].box, accel_r);
    }

    box_intersection_type right_box_isect;

    if (node.right_is_leaf()) {
      auto right_isect = intersect_primitive(
        intersector, primitives, node.right_leaf_index(), ray);
      if (right_isect.info < closest.info)
        closest = right_isect;
    } else {
      right_box_isect = detail::intersect(bvh_[node.right].box, accel_r);
    }

    if (left_box_isect && right_box_isect) {
      if (left_box_isect < right_box_isect) {
        stack.push(node.right, right_box_isect.tmin);
        stack.push(node.left, left_box_isect.tmin);
      } else {
        stack.push(node.left, left_box_isect.tmin);
        stack.push(node.right, right_box_isect.tmin);
      }
    } else if (left_box_isect) {
      stack.push(node.left, left_box_isect.tmin);
    } else if (right_box_isect) {
      stack.push(node.right, right_box_isect.tmin);
    }
  }

  return closest;
}

} // namespace lbvh

#ifndef LBVH_NO_TRIANGLE

namespace lbvh {

template<typename scalar_type>
class triangle_intersector;

template<typename scalar_type>
class triangle_aabb_converter;

//! Represents a 3D triangle from an .obj model.
//!
//! \tparam scalar_type The floating point type to represent the triangle with.
template<typename scalar_type>
class triangle final
{
public:
  using vec3_type = vec3<scalar_type>;

  constexpr triangle() = default;

  constexpr triangle(const vec3_type& a, const vec3_type& b, const vec3_type& c)
    : p0(a)
    , e0(math::operator-(a, b))
    , e1(math::operator-(c, a))
    , normal(math::cross(e0, e1))
  {}

private:
  friend triangle_intersector<scalar_type>;

  friend triangle_aabb_converter<scalar_type>;

  vec3_type p0;
  vec3_type e0;
  vec3_type e1;
  vec3_type normal;
};

//! Used for converting triangles in the model to bounding boxes.
//!
//! \tparam scalar_type The scalar type of the bounding box vectors to make.
template<typename scalar_type>
class triangle_aabb_converter final
{
public:
  //! A type definition for a model bouding box.
  using box_type = aabb<scalar_type>;
  //! A type definition for a triangle.
  using triangle_type = triangle<scalar_type>;
  //! Gets a bounding box for a triangle in the model.
  //!
  //! \param t The triangle to get the bounding box for.
  //!
  //! \return The bounding box for the specified triangle.
  box_type operator()(const triangle_type& t) const noexcept
  {
    using namespace math;

    const auto p1 = t.p0 - t.e0;
    const auto p2 = t.p0 + t.e1;

    auto tmp_min = min(t.p0, p1);
    auto tmp_max = max(t.p0, p1);

    return box_type{ min(tmp_min, p2), max(tmp_max, p2) };
  }
};

template<typename scalar_type>
struct triangle_intersection final
{
  using vec2_type = vec2<scalar_type>;

  using vec3_type = vec3<scalar_type>;

  scalar_type distance = std::numeric_limits<scalar_type>::infinity();

  vec2_type uv;

  constexpr bool operator<(const triangle_intersection& other) const noexcept
  {
    return distance < other.distance;
  }

  constexpr bool operator<(scalar_type other_distance) const noexcept
  {
    return distance < other_distance;
  }
};

//! Used to detect intersections between rays and triangles.
//!
//! \tparam scalar_type The scalar type of the triangle vector components.
template<typename scalar_type>
class triangle_intersector final
{
public:
  //! A type definition for a 2D vector.
  using vec2_type = vec2<scalar_type>;
  //! A type definition for a 3D vector.
  using vec3_type = vec3<scalar_type>;
  //! A type definition for a triangle.
  using triangle_type = triangle<scalar_type>;
  //! A type definition for an intersection.
  using intersection_type = triangle_intersection<scalar_type>;
  //! A type definition for a ray.
  using ray_type = ray<scalar_type>;
  //! Detects intersection between a ray and the triangle.
  intersection_type operator()(const triangle<scalar_type>& tri,
                               const ray_type& ray) const noexcept
  {
    using namespace math;

    const vec3_type c = tri.p0 - ray.pos;
    const vec3_type r = math::cross(ray.dir, c);
    const scalar_type inv_det = 1 / math::dot(tri.normal, ray.dir);

    const scalar_type u = dot(r, tri.e1) * inv_det;
    const scalar_type v = dot(r, tri.e0) * inv_det;
    const scalar_type w = scalar_type(1) - u - v;

    if ((u >= 0) && (v >= 0) && (w >= 0)) {
      const scalar_type t = dot(tri.normal, c) * inv_det;
      if ((t >= ray.tmin) && (t <= ray.tmax))
        return intersection_type{ t, { u, v } };
    }

    return intersection_type{};
  }
};

} // namespace lbvh

#endif // LBVH_NO_TRIANGLE
