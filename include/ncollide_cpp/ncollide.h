/**
 * ncollide.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: April 24, 2019
 * Authors: Toki Migimatsu
 */

#ifndef NCOLLIDE_CPP_NCOLLIDE_H_
#define NCOLLIDE_CPP_NCOLLIDE_H_

#include "ncollide2d.h"
#include "ncollide3d.h"

// namespace ncollide {

//   template<int Dim>
//   struct bounding_volume;

//   template<int Dim>
//   struct shape;

//   template<int Dim>
//   struct query;

// }  // namespace ncollide

template<int Dim>
struct ncollide {

  struct bounding_volume;
  struct shape;
  struct query;

};

// template<>
// struct ncollide<2>::bounding_volume {

//   using AABB = ncollide2d::bounding_volume::AABB;
//   static constexpr auto& aabb = ncollide2d::bounding_volume::aabb;

// };

template<>
struct ncollide<3>::bounding_volume {

  using AABB = ncollide3d::bounding_volume::AABB;
  static constexpr auto& aabb = ncollide3d::bounding_volume::aabb;

};

template<>
struct ncollide<2>::query {

  using PointProjection = ncollide2d::query::PointProjection;
  // using Ray = ncollide2d::query::Ray;
  // using RayIntersection = ncollide2d::query::RayIntersection;
  using ClosestPoints = ncollide2d::query::ClosestPoints;
  using Contact = ncollide2d::query::Contact;
  using Proximity = ncollide2d::query::Proximity;

  static constexpr auto& closest_points = ncollide2d::query::closest_points;
  static constexpr auto& contact = ncollide2d::query::contact;
  static constexpr auto& distance = ncollide2d::query::distance;
  static constexpr auto& proximity = ncollide2d::query::proximity;
  static constexpr auto& time_of_impact = ncollide2d::query::time_of_impact;

};

template<>
struct ncollide<3>::query {

  using PointProjection = ncollide3d::query::PointProjection;
  using Ray = ncollide3d::query::Ray;
  using RayIntersection = ncollide3d::query::RayIntersection;
  using ClosestPoints = ncollide3d::query::ClosestPoints;
  using Contact = ncollide3d::query::Contact;
  using Proximity = ncollide3d::query::Proximity;

  static constexpr auto& closest_points = ncollide3d::query::closest_points;
  static constexpr auto& contact = ncollide3d::query::contact;
  static constexpr auto& distance = ncollide3d::query::distance;
  static constexpr auto& proximity = ncollide3d::query::proximity;
  static constexpr auto& time_of_impact = ncollide3d::query::time_of_impact;

};

template<>
struct ncollide<2>::shape {

  using Shape = ncollide2d::shape::Shape;
  using ShapeVector = ncollide2d::shape::ShapeVector;
  using Ball = ncollide2d::shape::Ball;
  using Capsule = ncollide2d::shape::Capsule;
  using Compound = ncollide2d::shape::Compound;
  using ConvexPolygon = ncollide2d::shape::ConvexPolygon;
  using Cuboid = ncollide2d::shape::Cuboid;
  // using RoundedCuboid = ncollide2d::shape::RoundedCuboid;

};

template<>
struct ncollide<3>::shape {

  using Shape = ncollide3d::shape::Shape;
  using ShapeVector = ncollide3d::shape::ShapeVector;
  using Ball = ncollide3d::shape::Ball;
  using Capsule = ncollide3d::shape::Capsule;
  using Compound = ncollide3d::shape::Compound;
  using Cuboid = ncollide3d::shape::Cuboid;
  using RoundedCuboid = ncollide3d::shape::RoundedCuboid;
  using TriMesh = ncollide3d::shape::TriMesh;

};

#endif  // NCOLLIDE_CPP_NCOLLIDE_H_
