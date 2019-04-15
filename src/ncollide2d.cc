/**
 * ncollide2d.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 12, 2019
 * Authors: Toki Migimatsu
 */

#include "ncollide/ncollide2d.h"

#include "ncollide/ncollide_ffi.h"

namespace {

ncollide2d_math_isometry_t ConvertIsometry(const Eigen::Isometry2d& T) {
  ncollide2d_math_isometry_t result;
  Eigen::Map<Eigen::Vector2d>(result.translation) = T.translation();
  Eigen::Rotation2Dd rot(T.linear());
  result.angle = rot.angle();
  return result;
}

}  // namespace

namespace ncollide2d {
namespace shape {

Shape::Shape(ncollide2d_shape_t* ptr) : ptr_(ptr, ncollide2d_shape_delete) {}

void Shape::set_ptr(ncollide2d_shape_t* ptr) {
  ptr_ = std::shared_ptr<ncollide2d_shape_t>(ptr, ncollide2d_shape_delete);
}

query::point_internal::PointProjection Shape::project_point(const Eigen::Isometry2d& m,
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

Cuboid::Cuboid(const Eigen::Vector2d& half_extents)
    : Shape(ncollide2d_shape_cuboid_new(half_extents(0), half_extents(1))) {}

Cuboid::Cuboid(double x, double y)
    : Shape(ncollide2d_shape_cuboid_new(x, y)) {}

Eigen::Map<const Eigen::Vector2d> Cuboid::half_extents() const {
  return Eigen::Map<const Eigen::Vector2d>(ncollide2d_shape_cuboid_half_extents(ptr()));
}

Ball::Ball(double radius) : Shape(ncollide2d_shape_ball_new(radius)) {}

double Ball::radius() const {
  return ncollide2d_shape_ball_radius(ptr());
}

Capsule::Capsule(double half_height, double radius)
    : Shape(ncollide2d_shape_capsule_new(half_height, radius)) {}

double Capsule::half_height() const {
  return ncollide2d_shape_capsule_half_height(ptr());
}

double Capsule::radius() const {
  return ncollide2d_shape_capsule_radius(ptr());
}

Compound::Compound(const std::vector<std::pair<Eigen::Isometry2d, std::unique_ptr<Shape>>>& shapes) {
  std::vector<ncollide2d_math_isometry_t> transforms;
  std::vector<const ncollide2d_shape_t*> raw_shapes;
  transforms.reserve(shapes.size());
  raw_shapes.reserve(shapes.size());
  for (const std::pair<Eigen::Isometry2d, std::unique_ptr<Shape>>& shape : shapes) {
    transforms.push_back(ConvertIsometry(shape.first));
    raw_shapes.push_back(shape.second->ptr());
  }
  set_ptr(ncollide2d_shape_compound_new(transforms.data(), raw_shapes.data(), shapes.size()));
}

ConvexPolygon::ConvexPolygon(const std::vector<double[2]>& points)
    : Shape(ncollide2d_shape_convex_polygon_try_from_points(points.data(), points.size())) {}

}  // namespace shape

namespace query {

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

double distance(const Eigen::Isometry2d& m1, const shape::Shape& g1,
                const Eigen::Isometry2d& m2, const shape::Shape& g2) {
  ncollide2d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide2d_math_isometry_t c_m2 = ConvertIsometry(m2);

  return ncollide2d_query_distance(&c_m1, g1.ptr(), &c_m2, g2.ptr());
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

Proximity proximity(const Eigen::Isometry2d& m1, const shape::Shape& g1,
                    const Eigen::Isometry2d& m2, const shape::Shape& g2, double margin) {
  ncollide2d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide2d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ncollide2d_query_proximity_t result = ncollide2d_query_proximity(&c_m1, g1.ptr(),
                                                                   &c_m2, g2.ptr(), margin);

  switch (result) {
    case ncollide2d_query_proximity_Intersecting: return Proximity::Intersecting;
    case ncollide2d_query_proximity_WithinMargin: return Proximity::WithinMargin;
    case ncollide2d_query_proximity_Disjoint: return Proximity::Disjoint;
  }
}

std::optional<double> time_of_impact(const Eigen::Isometry2d& m1, const Eigen::Vector2d& v1,
                                     const shape::Shape& g1,
                                     const Eigen::Isometry2d& m2, const Eigen::Vector2d& v2,
                                     const shape::Shape& g2) {
  ncollide2d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide2d_math_isometry_t c_m2 = ConvertIsometry(m2);

  double out_time = 0.;
  bool result = ncollide2d_query_time_of_impact(&c_m1, v1.data(), g1.ptr(),
                                                &c_m2, v2.data(), g2.ptr(), &out_time);

  std::optional<double> time;
  if (result) {
    time = out_time;
  }
  return time;
}

}  // namespace query
}  // namespace ncollide2d
