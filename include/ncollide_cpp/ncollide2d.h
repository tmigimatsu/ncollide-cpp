/**
 * ncollide2d.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

#ifndef NCOLLIDE_CPP_NCOLLIDE2D_H_
#define NCOLLIDE_CPP_NCOLLIDE2D_H_

#include <memory>   // std::shared_ptr, std::unique_ptr
#include <utility>  // std::pair
#include <vector>   // std::vector

#if __cplusplus > 201402L
#include <optional>  // std::optional
#else
#include <ctrl_utils/optional.h>
#endif

#include <Eigen/Eigen>

struct ncollide2d_shape_t;
struct ncollide2d_bounding_volume_aabb_t;
struct ncollide2d_bounding_volume_bounding_sphere_t;
struct ncollide2d_query_ray_t;

namespace ncollide2d {
namespace shape {

class Shape;

}  // namespace shape

namespace bounding_volume {

class AABB {

 public:

  AABB(ncollide2d_bounding_volume_aabb_t* ptr);

  const ncollide2d_bounding_volume_aabb_t* ptr() const { return ptr_.get(); }
  ncollide2d_bounding_volume_aabb_t* ptr() { return ptr_.get(); }
  void set_ptr(ncollide2d_bounding_volume_aabb_t* ptr);

  Eigen::Map<const Eigen::Vector2d> maxs() const;

  Eigen::Map<const Eigen::Vector2d> mins() const;

 private:

  std::shared_ptr<ncollide2d_bounding_volume_aabb_t> ptr_;

};

class BoundingSphere {

 public:

  BoundingSphere(ncollide2d_bounding_volume_bounding_sphere_t* ptr);

  const ncollide2d_bounding_volume_bounding_sphere_t* ptr() const { return ptr_.get(); }
  ncollide2d_bounding_volume_bounding_sphere_t* ptr() { return ptr_.get(); }
  void set_ptr(ncollide2d_bounding_volume_bounding_sphere_t* ptr);

  double radius() const;

 private:

  std::shared_ptr<ncollide2d_bounding_volume_bounding_sphere_t> ptr_;

};

/**
 * Computes the axis-aligned bounding box of a shape g transformed by m.
 *
 * Same as g.aabb(m).
 */
AABB aabb(const shape::Shape& g, const Eigen::Isometry2d& m = Eigen::Isometry2d::Identity());

/**
 * Computes the bounding sphere of a shape g transformed by m.
 *
 * Same as g.bounding_sphere(m).
 */
BoundingSphere bounding_sphere(const shape::Shape& g,
                               const Eigen::Isometry2d& m = Eigen::Isometry2d::Identity());

}  // namespace bounding_volume

namespace query {

/**
 * Point queries
 */

struct PointProjection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool is_inside;
  Eigen::Vector2d point;
};

/**
 * Ray casting
 */

class Ray {

 public:

  Ray() = default;
  Ray(ncollide2d_query_ray_t* ptr);

  /**
   * Creates a new ray starting from origin and with the direction dir.
   * dir must be normalized.
   */
  Ray(Eigen::Ref<const Eigen::Vector2d> origin, Eigen::Ref<const Eigen::Vector2d> dir);

  const ncollide2d_query_ray_t* ptr() const { return ptr_.get(); }
  ncollide2d_query_ray_t* ptr() { return ptr_.get(); }
  void set_ptr(ncollide2d_query_ray_t* ptr);

  Eigen::Map<const Eigen::Vector2d> origin() const;
  Eigen::Map<const Eigen::Vector2d> dir() const;

  Eigen::Vector2d point_at(double t) const;

 private:

  std::shared_ptr<ncollide2d_query_ray_t> ptr_;

};

struct RayIntersection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double toi;
  Eigen::Vector2d normal;

};

/**
 * Pairwise queries
 */

struct ClosestPoints {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class Status {
    Intersecting,
    WithinMargin,
    Disjoint
  };

  Status status;
  Eigen::Vector2d point1;
  Eigen::Vector2d point2;
};

struct Contact {

  Contact() = default;
  Contact(Eigen::Ref<const Eigen::Vector2d> world1, Eigen::Ref<const Eigen::Vector2d> world2,
          Eigen::Ref<const Eigen::Vector2d> normal, double depth)
      : world1(world1), world2(world2), normal(normal), depth(depth) {}

  Eigen::Vector2d world1;
  Eigen::Vector2d world2;
  Eigen::Vector2d normal;
  double depth;

};

struct TOI {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class Status {
    OutOfIterations,
    Converged,
    Failed,
    Penetrating
  };

  TOI() = default;
  TOI(double toi, Eigen::Ref<const Eigen::Vector2d> witness1,
      Eigen::Ref<const Eigen::Vector2d> witness2,
      Eigen::Ref<const Eigen::Vector2d> normal1,
      Eigen::Ref<const Eigen::Vector2d> normal2,
      Status status)
      : toi(toi), witness1(witness1), witness2(witness2), normal1(normal1),
        normal2(normal2), status(status) {}

  double toi;
  Eigen::Vector2d witness1;
  Eigen::Vector2d witness2;
  Eigen::Vector2d normal1;
  Eigen::Vector2d normal2;
  Status status;
};

enum class Proximity {
  Intersecting,
  WithinMargin,
  Disjoint
};

/**
 * Computes the pair of closest points between two shapes.
 *
 * Returns None if the objects are separated by a distance greater than
 * max_dist.
 */
ClosestPoints closest_points(const Eigen::Isometry2d& m1, const shape::Shape& g1,
                             const Eigen::Isometry2d& m2, const shape::Shape& g2,
                             double max_dist);

/**
 * Computes one contact point between two shapes.
 *
 * Returns None if the objects are separated by a distance greater than prediction.
 */
std::optional<Contact> contact(const Eigen::Isometry2d& m1, const shape::Shape& g1,
                               const Eigen::Isometry2d& m2, const shape::Shape& g2,
                               double prediction);

/**
 * Computes the minimum distance separating two shapes.
 *
 * Returns `0.0` if the objects are touching or penetrating.
 */
double distance(const Eigen::Isometry2d& m1, const shape::Shape& g1,
                const Eigen::Isometry2d& m2, const shape::Shape& g2);

/**
 * Tests whether two shapes are in intersecting or separated by a distance
 * smaller than margin.
 */
Proximity proximity(const Eigen::Isometry2d& m1, const shape::Shape& g1,
                    const Eigen::Isometry2d& m2, const shape::Shape& g2,
                    double margin);

/**
 * Computes the smallest time of impact of two shapes under translational
 * movement.
 *
 * Returns 0.0 if the objects are touching or penetrating.
 */
std::optional<TOI> time_of_impact(const Eigen::Isometry2d& m1, const Eigen::Vector2d& v1,
                                  const shape::Shape& g1,
                                  const Eigen::Isometry2d& m2, const Eigen::Vector2d& v2,
                                  const shape::Shape& g2,
                                  double max_toi, double target_distance);

}  // namespace query

namespace shape {

class Shape {

 public:

  Shape() : ptr_(nullptr) {}
  Shape(ncollide2d_shape_t* ptr);

  virtual ~Shape() = default;

  const ncollide2d_shape_t* ptr() const { return ptr_.get(); }
  ncollide2d_shape_t* ptr() { return ptr_.get(); }
  void set_ptr(ncollide2d_shape_t* ptr);

  /**
   * The AABB of self.
   */
  bounding_volume::AABB aabb(const Eigen::Isometry2d& m = Eigen::Isometry2d::Identity()) const;

  /**
   * The bounding sphere of self.
   */
  bounding_volume::BoundingSphere bounding_sphere(const Eigen::Isometry2d& m = Eigen::Isometry2d::Identity()) const;

  query::PointProjection project_point(const Eigen::Isometry2d& m,
                                       const Eigen::Vector2d& pt, bool solid) const;

  double distance_to_point(const Eigen::Isometry2d& m, const Eigen::Vector2d& pt, bool solid) const;

  bool contains_point(const Eigen::Isometry2d& m, const Eigen::Vector2d& pt) const;

  /**
   * Computes the time of impact between this transformed shape and a ray.
   */
  std::optional<double> toi_with_ray(const Eigen::Isometry2d& m, const query::Ray& ray,
                                     double max_toi, bool solid) const;

  /**
   * Computes the time of impact and normal between this transformed shape and a ray.
   */
  std::optional<query::RayIntersection> toi_and_normal_with_ray(const Eigen::Isometry2d& m,
                                                                const query::Ray& ray,
                                                                double max_toi,
                                                                bool solid) const;

 private:

  std::shared_ptr<ncollide2d_shape_t> ptr_;

};

using ShapeVector = std::vector<std::pair<Eigen::Isometry2d, std::unique_ptr<Shape>>>;

class Ball : public Shape {

 public:

  Ball(double radius);

  double radius() const;

};

class Capsule : public Shape {

 public:

  Capsule(double half_height, double radius);

  double half_height() const;
  double radius() const;

};

class Compound : public Shape {

 public:

  Compound(ShapeVector&& shapes);

  const ShapeVector& shapes() const { return shapes_; }

 private:

  ShapeVector shapes_;

};

class ConvexPolygon : public Shape {

 public:

  ConvexPolygon(const std::vector<std::array<double, 2>>& points);

};

class Cuboid : public Shape {

 public:

  Cuboid(const Eigen::Vector2d& half_extents);
  Cuboid(double x, double y);

  Eigen::Map<const Eigen::Vector2d> half_extents() const;

};

}  // namespace shape
}  // namespace ncollide2d

#endif  // NCOLLIDE_CPP_NCOLLIDE2D_H_
