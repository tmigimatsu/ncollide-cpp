/**
 * ncollide2d.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 12, 2019
 * Authors: Toki Migimatsu
 */

#include "ncollide_cpp/ncollide2d.h"

#include "ncollide_cpp/ncollide_ffi.h"

namespace {

ncollide2d_math_isometry_t ConvertIsometry(const Eigen::Isometry2d& T) {
  ncollide2d_math_isometry_t result;
  Eigen::Map<Eigen::Vector2d>(result.translation) = T.translation();
  Eigen::Rotation2Dd rot(T.linear());
  result.rotation = rot.angle();
  return result;
}
ncollide2d::query::TOI::Status ConvertTOIStatus(ncollide_query_toi_status_t status) {
  switch (status) {
    case ncollide_query_toi_OutOfIterations:
      return ncollide2d::query::TOI::Status::OutOfIterations;
    case ncollide_query_toi_Converged:
      return ncollide2d::query::TOI::Status::Converged;
    case ncollide_query_toi_Failed:
      return ncollide2d::query::TOI::Status::Failed;
    case ncollide_query_toi_Penetrating:
      return ncollide2d::query::TOI::Status::Penetrating;
  }
  throw std::invalid_argument("ConvertTOIStatus(): Invalid status.");
}

}  // namespace

namespace ncollide2d {
namespace bounding_volume {

AABB::AABB(ncollide2d_bounding_volume_aabb_t* ptr)
    : ptr_(ptr, ncollide2d_bounding_volume_aabb_delete) {}

Eigen::Map<const Eigen::Vector2d> AABB::maxs() const {
  return Eigen::Map<const Eigen::Vector2d>(ncollide2d_bounding_volume_aabb_maxs(ptr()));
}

Eigen::Map<const Eigen::Vector2d> AABB::mins() const {
  return Eigen::Map<const Eigen::Vector2d>(ncollide2d_bounding_volume_aabb_mins(ptr()));
}

AABB aabb(const shape::Shape& g, const Eigen::Isometry2d& m) {
  ncollide2d_math_isometry_t c_m = ConvertIsometry(m);
  return AABB(ncollide2d_bounding_volume_aabb(g.ptr(), &c_m));
}

BoundingSphere::BoundingSphere(ncollide2d_bounding_volume_bounding_sphere_t* ptr)
    : ptr_(ptr, ncollide2d_bounding_volume_bounding_sphere_delete) {}

BoundingSphere bounding_sphere(const shape::Shape& g, const Eigen::Isometry2d& m) {
  ncollide2d_math_isometry_t c_m = ConvertIsometry(m);
  return BoundingSphere(ncollide2d_bounding_volume_bounding_sphere(g.ptr(), &c_m));
}

double BoundingSphere::radius() const {
  return ncollide2d_bounding_volume_bounding_sphere_radius(ptr());
}

}  // namespace bounding_volume

namespace query {

Ray::Ray(ncollide2d_query_ray_t* ptr) : ptr_(ptr, ncollide2d_query_ray_delete) {}

Ray::Ray(Eigen::Ref<const Eigen::Vector2d> origin, Eigen::Ref<const Eigen::Vector2d> dir)
    : ptr_(ncollide2d_query_ray_new(origin.data(), dir.data()), ncollide2d_query_ray_delete) {}

void Ray::set_ptr(ncollide2d_query_ray_t* ptr) {
  ptr_ = std::shared_ptr<ncollide2d_query_ray_t>(ptr, ncollide2d_query_ray_delete);
}

Eigen::Map<const Eigen::Vector2d> Ray::origin() const {
  return Eigen::Map<const Eigen::Vector2d>(ncollide2d_query_ray_origin(ptr()));
}

Eigen::Map<const Eigen::Vector2d> Ray::dir() const {
  return Eigen::Map<const Eigen::Vector2d>(ncollide2d_query_ray_dir(ptr()));
}

Eigen::Vector2d Ray::point_at(double t) const {
  return origin() + t * dir();
}

ClosestPoints closest_points(const Eigen::Isometry2d& m1, const shape::Shape& g1,
                             const Eigen::Isometry2d& m2, const shape::Shape& g2,
                             double max_dist) {
  ncollide2d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide2d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ClosestPoints points;
  auto result = ncollide2d_query_closest_points(&c_m1, g1.ptr(), &c_m2, g2.ptr(), max_dist,
                                                points.point1.data(), points.point2.data());

  switch (result) {
    case ncollide2d_query_closest_points_Intersecting:
      points.status = ClosestPoints::Status::Intersecting;
      break;
    case ncollide2d_query_closest_points_WithinMargin:
      points.status = ClosestPoints::Status::WithinMargin;
      break;
    case ncollide2d_query_closest_points_Disjoint:
      points.status = ClosestPoints::Status::Disjoint;
      break;
  }
  return points;
}

std::optional<Contact> contact(const Eigen::Isometry2d& m1, const shape::Shape& g1,
                               const Eigen::Isometry2d& m2, const shape::Shape& g2,
                               double prediction) {
  ncollide2d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide2d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ncollide2d_query_contact_t out_contact;
  bool result = ncollide2d_query_contact(&c_m1, g1.ptr(), &c_m2, g2.ptr(), prediction, &out_contact);

  std::optional<Contact> contact;
  if (result) {
    contact.emplace(Eigen::Ref<const Eigen::Vector2d>(Eigen::Map<Eigen::Vector2d>(out_contact.world1)),
                    Eigen::Ref<const Eigen::Vector2d>(Eigen::Map<Eigen::Vector2d>(out_contact.world2)),
                    Eigen::Ref<const Eigen::Vector2d>(Eigen::Map<Eigen::Vector2d>(out_contact.normal)),
                    out_contact.depth);
  }
  return contact;
}

double distance(const Eigen::Isometry2d& m1, const shape::Shape& g1,
                const Eigen::Isometry2d& m2, const shape::Shape& g2) {
  ncollide2d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide2d_math_isometry_t c_m2 = ConvertIsometry(m2);

  return ncollide2d_query_distance(&c_m1, g1.ptr(), &c_m2, g2.ptr());
}

Proximity proximity(const Eigen::Isometry2d& m1, const shape::Shape& g1,
                    const Eigen::Isometry2d& m2, const shape::Shape& g2, double margin) {
  ncollide2d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide2d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ncollide2d_query_proximity_t result = ncollide2d_query_proximity(&c_m1, g1.ptr(),
                                                                   &c_m2, g2.ptr(), margin);

  switch (result) {
    case ncollide2d_query_proximity_Intersecting: return Proximity::Intersecting;
    case ncollide2d_query_proximity_WithinMargin: return Proximity::WithinMargin;
    default: return Proximity::Disjoint;
  }
}

std::optional<TOI> time_of_impact(const Eigen::Isometry2d& m1, const Eigen::Vector2d& v1,
                                  const shape::Shape& g1,
                                  const Eigen::Isometry2d& m2, const Eigen::Vector2d& v2,
                                  const shape::Shape& g2, double max_toi,
                                  double target_distance) {
  ncollide2d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide2d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ncollide2d_query_toi_t out_toi;
  bool result = ncollide2d_query_time_of_impact(&c_m1, v1.data(), g1.ptr(),
                                                &c_m2, v2.data(), g2.ptr(),
                                                max_toi, target_distance, &out_toi);

  std::optional<TOI> toi;
  if (result) {
    toi.emplace(out_toi.toi,
        Eigen::Ref<const Eigen::Vector2d>(Eigen::Map<Eigen::Vector2d>(out_toi.witness1)),
        Eigen::Ref<const Eigen::Vector2d>(Eigen::Map<Eigen::Vector2d>(out_toi.witness2)),
        Eigen::Ref<const Eigen::Vector2d>(Eigen::Map<Eigen::Vector2d>(out_toi.normal1)),
        Eigen::Ref<const Eigen::Vector2d>(Eigen::Map<Eigen::Vector2d>(out_toi.normal2)),
        ConvertTOIStatus(out_toi.status));
  }
  return toi;
}

}  // namespace query

namespace shape {

/**
 * Shape
 */

Shape::Shape(ncollide2d_shape_t* ptr) : ptr_(ptr, ncollide2d_shape_delete) {}

void Shape::set_ptr(ncollide2d_shape_t* ptr) {
  ptr_ = std::shared_ptr<ncollide2d_shape_t>(ptr, ncollide2d_shape_delete);
}

bounding_volume::AABB Shape::aabb(const Eigen::Isometry2d& m) const {
  return bounding_volume::aabb(*this, m);
}

bounding_volume::BoundingSphere Shape::bounding_sphere(const Eigen::Isometry2d& m) const {
  return bounding_volume::bounding_sphere(*this, m);
}

query::PointProjection Shape::project_point(const Eigen::Isometry2d& m,
                                            const Eigen::Vector2d& pt,
                                            bool solid) const {
  ncollide2d_math_isometry_t c_m = ConvertIsometry(m);
  ncollide2d_query_point_projection_t result;
  ncollide2d_query_project_point(ptr_.get(), &c_m, pt.data(), solid, &result);
  Eigen::Map<const Eigen::Vector2d> point(result.point);
  return { result.is_inside, point };
}

double Shape::distance_to_point(const Eigen::Isometry2d& m, const Eigen::Vector2d& pt,
                                bool solid) const {
  ncollide2d_math_isometry_t c_m = ConvertIsometry(m);
  return ncollide2d_query_distance_to_point(ptr_.get(), &c_m, pt.data(), solid);
}

bool Shape::contains_point(const Eigen::Isometry2d& m, const Eigen::Vector2d& pt) const {
  ncollide2d_math_isometry_t c_m = ConvertIsometry(m);
  return ncollide2d_query_contains_point(ptr_.get(), &c_m, pt.data());
}

std::optional<double> Shape::toi_with_ray(const Eigen::Isometry2d& m, const query::Ray& ray,
                                          double max_toi, bool solid) const {
  ncollide2d_math_isometry_t c_m = ConvertIsometry(m);
  double out_toi;
  bool result = ncollide2d_query_toi_with_ray(ptr(), &c_m, ray.ptr(), max_toi, solid, &out_toi);
  std::optional<double> toi;
  if (result) toi = out_toi;
  return toi;
}

std::optional<query::RayIntersection> Shape::toi_and_normal_with_ray(const Eigen::Isometry2d& m,
                                                                     const query::Ray& ray,
                                                                     double max_toi,
                                                                     bool solid) const {
  ncollide2d_math_isometry_t c_m = ConvertIsometry(m);
  ncollide2d_query_ray_intersection_t out_ray;
  bool result = ncollide2d_query_toi_and_normal_with_ray(ptr(), &c_m, ray.ptr(), max_toi, solid, &out_ray);
  std::optional<query::RayIntersection> intersect;
  if (result) {
    Eigen::Map<const Eigen::Vector2d> normal(out_ray.normal);
    intersect = { out_ray.toi, normal };
  }
  return intersect;
}

/**
 * Ball
 */

Ball::Ball(double radius) : Shape(ncollide2d_shape_ball_new(radius)) {}

double Ball::radius() const {
  return ncollide2d_shape_ball_radius(ptr());
}

/**
 * Capsule
 */

Capsule::Capsule(double half_height, double radius)
    : Shape(ncollide2d_shape_capsule_new(half_height, radius)) {}

double Capsule::half_height() const {
  return ncollide2d_shape_capsule_half_height(ptr());
}

double Capsule::radius() const {
  return ncollide2d_shape_capsule_radius(ptr());
}

/**
 * Compound
 */

Compound::Compound(ShapeVector&& shapes) : shapes_(std::move(shapes)) {
  std::vector<ncollide2d_math_isometry_t> transforms;
  std::vector<const ncollide2d_shape_t*> raw_shapes;
  transforms.reserve(shapes_.size());
  raw_shapes.reserve(shapes_.size());
  for (const std::pair<Eigen::Isometry2d, std::unique_ptr<Shape>>& shape : shapes_) {
    transforms.push_back(ConvertIsometry(shape.first));
    raw_shapes.push_back(shape.second->ptr());
  }
  set_ptr(ncollide2d_shape_compound_new(transforms.data(), raw_shapes.data(), shapes_.size()));
}

/**
 * Convex polygon
 */

ConvexPolygon::ConvexPolygon(const std::vector<std::array<double, 2>>& points)
    : Shape(ncollide2d_shape_convex_polygon_try_from_points(reinterpret_cast<const double(*)[2]>(points.data()), points.size())) {}

/**
 * Cuboid
 */

Cuboid::Cuboid(const Eigen::Vector2d& half_extents)
    : Shape(ncollide2d_shape_cuboid_new(half_extents(0), half_extents(1))) {}

Cuboid::Cuboid(double x, double y)
    : Shape(ncollide2d_shape_cuboid_new(x, y)) {}

Eigen::Map<const Eigen::Vector2d> Cuboid::half_extents() const {
  return Eigen::Map<const Eigen::Vector2d>(ncollide2d_shape_cuboid_half_extents(ptr()));
}

}  // namespace shape
}  // namespace ncollide2d
