/**
 * ncollide3d.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 12, 2019
 * Authors: Toki Migimatsu
 */

#include "ncollide_cpp/ncollide3d.h"

#include <exception>  // std::runtime_error
#include <limits>     // std::numeric_limits

#include "ncollide_cpp/ncollide_ffi.h"

namespace {

ncollide3d_math_isometry_t ConvertIsometry(const Eigen::Isometry3d& T) {
  ncollide3d_math_isometry_t result;
  Eigen::Map<Eigen::Vector3d>(result.translation) = T.translation();
  Eigen::AngleAxisd aa(T.linear());
  Eigen::Map<Eigen::Vector3d>(result.rotation) = aa.angle() * aa.axis();
  return result;
}

Eigen::Isometry2d Isometry3dto2d(const Eigen::Isometry3d& T) {
  const Eigen::AngleAxisd aa(T.linear());
  return Eigen::Translation2d(T.translation().head<2>()) * Eigen::Rotation2Dd(aa.angle());
}

}  // namespace

namespace ncollide3d {
namespace bounding_volume {

AABB::AABB(ncollide3d_bounding_volume_aabb_t* ptr)
    : ptr_(ptr, ncollide3d_bounding_volume_aabb_delete) {}

Eigen::Map<const Eigen::Vector3d> AABB::maxs() const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_bounding_volume_aabb_maxs(ptr()));
}

Eigen::Map<const Eigen::Vector3d> AABB::mins() const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_bounding_volume_aabb_mins(ptr()));
}

AABB aabb(const shape::Shape& g, const Eigen::Isometry3d& m) {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  return AABB(ncollide3d_bounding_volume_aabb(g.ptr(), &c_m));
}

BoundingSphere::BoundingSphere(ncollide3d_bounding_volume_bounding_sphere_t* ptr)
    : ptr_(ptr, ncollide3d_bounding_volume_bounding_sphere_delete) {}

BoundingSphere bounding_sphere(const shape::Shape& g, const Eigen::Isometry3d& m) {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  return BoundingSphere(ncollide3d_bounding_volume_bounding_sphere(g.ptr(), &c_m));
}

double BoundingSphere::radius() const {
  return ncollide3d_bounding_volume_bounding_sphere_radius(ptr());
}

}  // namespace bounding_volume

namespace query {

Ray::Ray(ncollide3d_query_ray_t* ptr) : ptr_(ptr, ncollide3d_query_ray_delete) {}

Ray::Ray(Eigen::Ref<const Eigen::Vector3d> origin, Eigen::Ref<const Eigen::Vector3d> dir)
    : ptr_(ncollide3d_query_ray_new(origin.data(), dir.data()), ncollide3d_query_ray_delete) {}

void Ray::set_ptr(ncollide3d_query_ray_t* ptr) {
  ptr_ = std::shared_ptr<ncollide3d_query_ray_t>(ptr, ncollide3d_query_ray_delete);
}

Eigen::Map<const Eigen::Vector3d> Ray::origin() const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_query_ray_origin(ptr()));
}

Eigen::Map<const Eigen::Vector3d> Ray::dir() const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_query_ray_dir(ptr()));
}

Eigen::Vector3d Ray::point_at(double t) const {
  return origin() + t * dir();
}

ClosestPoints closest_points(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                             const Eigen::Isometry3d& m2, const shape::Shape& g2,
                             double max_dist) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ClosestPoints points;
  auto result = ncollide3d_query_closest_points(&c_m1, g1.ptr(), &c_m2, g2.ptr(), max_dist,
                                                points.point1.data(), points.point2.data());

  switch (result) {
    case ncollide3d_query_closest_points_Intersecting:
      points.status = ClosestPoints::Status::Intersecting;
      break;
    case ncollide3d_query_closest_points_WithinMargin:
      points.status = ClosestPoints::Status::WithinMargin;
      break;
    case ncollide3d_query_closest_points_Disjoint:
      points.status = ClosestPoints::Status::Disjoint;
      break;
  }
  return points;
}

std::optional<Contact> contact(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                               const Eigen::Isometry3d& m2, const shape::Shape& g2,
                               double prediction) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ncollide3d_query_contact_t out_contact;
  bool result = ncollide3d_query_contact(&c_m1, g1.ptr(), &c_m2, g2.ptr(), prediction, &out_contact);

  std::optional<Contact> contact;
  if (result) {
    contact.emplace(Eigen::Ref<const Eigen::Vector3d>(Eigen::Map<Eigen::Vector3d>(out_contact.world1)),
                    Eigen::Ref<const Eigen::Vector3d>(Eigen::Map<Eigen::Vector3d>(out_contact.world2)),
                    Eigen::Ref<const Eigen::Vector3d>(Eigen::Map<Eigen::Vector3d>(out_contact.normal)),
                    out_contact.depth);
  }
  return contact;
}

double distance(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                const Eigen::Isometry3d& m2, const shape::Shape& g2) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  return ncollide3d_query_distance(&c_m1, g1.ptr(), &c_m2, g2.ptr());
}

Proximity proximity(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                    const Eigen::Isometry3d& m2, const shape::Shape& g2, double margin) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ncollide3d_query_proximity_t result = ncollide3d_query_proximity(&c_m1, g1.ptr(),
                                                                   &c_m2, g2.ptr(), margin);

  switch (result) {
    case ncollide3d_query_proximity_Intersecting: return Proximity::Intersecting;
    case ncollide3d_query_proximity_WithinMargin: return Proximity::WithinMargin;
    default: return Proximity::Disjoint;
  }
}

std::optional<double> time_of_impact(const Eigen::Isometry3d& m1, const Eigen::Vector3d& v1,
                                     const shape::Shape& g1,
                                     const Eigen::Isometry3d& m2, const Eigen::Vector3d& v2,
                                     const shape::Shape& g2) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  double out_time = 0.;
  bool result = ncollide3d_query_time_of_impact(&c_m1, v1.data(), g1.ptr(),
                                                &c_m2, v2.data(), g2.ptr(), &out_time);

  std::optional<double> time;
  if (result) {
    time = out_time;
  }
  return time;
}

}  // namespace query

namespace shape {

/**
 * Shape
 */

Shape::Shape(ncollide3d_shape_t* ptr) : ptr_(ptr, ncollide3d_shape_delete) {}

void Shape::set_ptr(ncollide3d_shape_t* ptr) {
  ptr_ = std::shared_ptr<ncollide3d_shape_t>(ptr, ncollide3d_shape_delete);
}

bounding_volume::AABB Shape::aabb(const Eigen::Isometry3d& m) const {
  return bounding_volume::aabb(*this, m);
}

bounding_volume::BoundingSphere Shape::bounding_sphere(const Eigen::Isometry3d& m) const {
  return bounding_volume::bounding_sphere(*this, m);
}

query::PointProjection Shape::project_point(const Eigen::Isometry3d& m,
                                            const Eigen::Vector3d& pt, bool solid) const {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  ncollide3d_query_point_projection_t result;
  ncollide3d_query_project_point(ptr_.get(), &c_m, pt.data(), solid, &result);
  Eigen::Map<const Eigen::Vector3d> point(result.point);
  return { result.is_inside, point };
}

double Shape::distance_to_point(const Eigen::Isometry3d& m, const Eigen::Vector3d& pt,
                                bool solid) const {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  return ncollide3d_query_distance_to_point(ptr_.get(), &c_m, pt.data(), solid);
}

bool Shape::contains_point(const Eigen::Isometry3d& m, const Eigen::Vector3d& pt) const {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  return ncollide3d_query_contains_point(ptr_.get(), &c_m, pt.data());
}

std::optional<double> Shape::toi_with_ray(const Eigen::Isometry3d& m, const query::Ray& ray,
                                          bool solid) const {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  double out_toi;
  bool result = ncollide3d_query_toi_with_ray(ptr(), &c_m, ray.ptr(), solid, &out_toi);
  std::optional<double> toi;
  if (result) toi = out_toi;
  return toi;
}

std::optional<query::RayIntersection> Shape::toi_and_normal_with_ray(const Eigen::Isometry3d& m,
                                                                     const query::Ray& ray,
                                                                     bool solid) const {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  ncollide3d_query_ray_intersection_t out_ray;
  bool result = ncollide3d_query_toi_and_normal_with_ray(ptr(), &c_m, ray.ptr(), solid, &out_ray);
  std::optional<query::RayIntersection> intersect;
  if (result) {
    Eigen::Map<const Eigen::Vector3d> normal(out_ray.normal);
    intersect = { out_ray.toi, normal };
  }
  return intersect;
}

TriMesh Shape::to_trimesh() const {
  bounding_volume::AABB box = aabb();
  Cuboid cuboid((box.maxs() - box.mins()) / 2.);
  return cuboid.to_trimesh();
}

TriMesh Shape::convex_hull() const {
  TriMesh trimesh = to_trimesh();
  std::vector<std::array<double, 3>> points;
  for (size_t i = 0; i < trimesh.num_points(); i++) {
    const auto point = trimesh.point(i);
    points.push_back({point(0), point(1), point(2)});
  }
  return transformation::convex_hull(points);
}

/**
 * Ball
 */

Ball::Ball(double radius) : Shape(ncollide3d_shape_ball_new(radius)) {}

double Ball::radius() const {
  return ncollide3d_shape_ball_radius(ptr());
}

std::unique_ptr<ncollide2d::shape::Shape> Ball::project_2d() const {
  return std::make_unique<ncollide2d::shape::Ball>(radius());
}

/**
 * Capsule
 */

Capsule::Capsule(double half_height, double radius)
    : Shape(ncollide3d_shape_capsule_new(half_height, radius)) {}

double Capsule::half_height() const {
  return ncollide3d_shape_capsule_half_height(ptr());
}
double Capsule::radius() const {
  return ncollide3d_shape_capsule_radius(ptr());
}

std::unique_ptr<ncollide2d::shape::Shape> Capsule::project_2d() const {
  return std::make_unique<ncollide2d::shape::Capsule>(half_height(), radius());
}

/**
 * Compound
 */

Compound::Compound(ShapeVector&& shapes) : shapes_(std::move(shapes)) {
  std::vector<ncollide3d_math_isometry_t> transforms;
  std::vector<const ncollide3d_shape_t*> raw_shapes;
  transforms.reserve(shapes_.size());
  raw_shapes.reserve(shapes_.size());
  for (const std::pair<Eigen::Isometry3d, std::unique_ptr<Shape>>& shape : shapes_) {
    transforms.push_back(ConvertIsometry(shape.first));
    raw_shapes.push_back(shape.second->ptr());
  }
  set_ptr(ncollide3d_shape_compound_new(transforms.data(), raw_shapes.data(), shapes_.size()));

  // TODO: Replace shapes_ with new shapes created in rust
}

std::unique_ptr<ncollide2d::shape::Shape> Compound::project_2d() const {

  std::vector<std::pair<Eigen::Isometry2d, std::unique_ptr<ncollide2d::shape::Shape>>> shapes_2d;
  shapes_2d.reserve(shapes_.size());
  for (const auto& m_g : shapes_) {
    const Eigen::Isometry3d& m = m_g.first;
    const std::unique_ptr<Shape>& g = m_g.second;
    shapes_2d.push_back({ Isometry3dto2d(m), g->project_2d() });
  }

  // throw std::runtime_error("Compound::project_2d(): Not implemented yet.");
  return std::make_unique<ncollide2d::shape::Compound>(std::move(shapes_2d));
}

/**
 * Convex hull
 */

ConvexHull::ConvexHull(const std::vector<std::array<double, 3>>& points)
  : Shape(ncollide3d_shape_convex_hull_try_from_points(reinterpret_cast<const double(*)[3]>(points.data()), points.size())) {}

std::unique_ptr<ncollide2d::shape::Shape> ConvexHull::project_2d() const {
  throw std::runtime_error("ConvexHull::project_2d(): Not implemented yet.");
  return std::make_unique<ncollide2d::shape::ConvexPolygon>(std::vector<std::array<double, 2>>({}));
}

size_t ConvexHull::num_points() const {
  return ncollide3d_shape_convex_hull_num_points(ptr());
}

Eigen::Map<const Eigen::Vector3d> ConvexHull::point(size_t i) const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_shape_convex_hull_point(ptr(), i));
}

/**
 * Cuboid
 */

Cuboid::Cuboid(const Eigen::Vector3d& half_extents)
    : Shape(ncollide3d_shape_cuboid_new(half_extents(0), half_extents(1), half_extents(2))) {}

Cuboid::Cuboid(double x, double y, double z)
    : Shape(ncollide3d_shape_cuboid_new(x, y, z)) {}

Eigen::Map<const Eigen::Vector3d> Cuboid::half_extents() const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_shape_cuboid_half_extents(ptr()));
}

std::unique_ptr<ncollide2d::shape::Shape> Cuboid::project_2d() const {
  Eigen::Vector3d h = half_extents();
  return std::make_unique<ncollide2d::shape::Cuboid>(h(0), h(1));
}

TriMesh Cuboid::to_trimesh() const {
  return ncollide3d_shape_cuboid_to_trimesh(ptr());
}

/**
 * Rounded cuboid
 */

RoundedCuboid::RoundedCuboid(const Eigen::Vector3d& half_extents, double radius)
    : Shape(ncollide3d_shape_rounded_cuboid_new(half_extents(0), half_extents(1), half_extents(2),
                                                radius)) {}

RoundedCuboid::RoundedCuboid(double x, double y, double z, double radius)
    : Shape(ncollide3d_shape_rounded_cuboid_new(x, y, z, radius)) {}

Eigen::Map<const Eigen::Vector3d> RoundedCuboid::half_extents() const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_shape_rounded_cuboid_half_extents(ptr()));
}

double RoundedCuboid::radius() const {
  return ncollide3d_shape_rounded_cuboid_radius(ptr());
}

std::unique_ptr<ncollide2d::shape::Shape> RoundedCuboid::project_2d() const {
  const auto h = half_extents();
  return std::make_unique<ncollide2d::shape::Cuboid>(h(0), h(1));
  // const double r = radius();
  // return std::make_unique<ncollide2d::shape::Cuboid>(h(0) + r, h(1) + r);
}

/**
 * Cylinder
 */

// Cylinder::Cylinder(double half_height, double radius)
//     : Shape(ncollide3d_shape_capsule_new(half_height, radius)) {}

// double Cylinder::half_height() const {
//   return ncollide3d_shape_capsule_half_height(ptr());
// }
// double Cylinder::radius() const {
//   return ncollide3d_shape_capsule_radius(ptr());
// }

// std::unique_ptr<ncollide2d::shape::Shape> Cylinder::project_2d() const {
//   return std::make_unique<ncollide2d::shape::Ball>(radius());
// }

/**
 * Trimesh
 */

TriMesh::TriMesh(const std::string& filename)
    : Shape(ncollide3d_shape_trimesh_file(filename.c_str())) {}

TriMesh::TriMesh(const std::vector<double[3]>& points, const std::vector<size_t[3]>& indices)
    : Shape(ncollide3d_shape_trimesh_new(points.data(), points.size(),
                                         indices.data(), indices.size())) {}

size_t TriMesh::num_points() const {
  return ncollide3d_shape_trimesh_num_points(ptr());
}

Eigen::Map<const Eigen::Vector3d> TriMesh::point(size_t i) const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_shape_trimesh_point(ptr(), i));
}

std::unique_ptr<ncollide2d::shape::Shape> TriMesh::project_2d() const {
  throw std::runtime_error("TriMesh::project_2d(): Not implemented yet.");
  return std::make_unique<ncollide2d::shape::Ball>(0.);
}

}  // namespace shape

namespace transformation {

shape::TriMesh convex_hull(const std::vector<std::array<double, 3>>& points) {
  const auto arr_points = reinterpret_cast<const double(*)[3]>(points.data());
  return ncollide3d_transformation_convex_hull(arr_points, points.size());
}

}  // namespace transformation
}  // namespace ncollide3d
