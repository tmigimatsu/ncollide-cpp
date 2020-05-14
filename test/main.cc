/**
 * main.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: April 18, 2019
 * Authors: Toki Migimatsu
 */

#include <cmath>     // std::sin, std::cos
#include <iostream>  // std::cout
#include <fstream>  // std::cout

#include <ncollide_cpp/ncollide.h>

namespace {

const double kRadius = 0.2;
const size_t kNumGrid = 50;

const double kShapeSize = 0.;

}  // namespace

int main(int argc, char *argv[]) {

  const ncollide3d::shape::RoundedCuboid shape(kShapeSize, kShapeSize, kShapeSize, 0.05);

  std::ofstream file("output");

  const auto ProjectFrom = [&shape, &file](const Eigen::Vector3d& xyz) {
    const ncollide3d::query::Ray ray_outer(xyz, -xyz.normalized());
    const ncollide3d::query::Ray ray_inner(Eigen::Vector3d::Zero(), xyz.normalized());
    auto intersect_outer = shape.toi_and_normal_with_ray(Eigen::Isometry3d::Identity(), ray_outer, 100, false);
    auto intersect_inner = shape.toi_and_normal_with_ray(Eigen::Isometry3d::Identity(), ray_inner, 100, false);

    const Eigen::Vector3d point_outer = ray_outer.origin() + intersect_outer->toi * ray_outer.dir();
    const Eigen::Vector3d point_inner = ray_inner.origin() + intersect_inner->toi * ray_inner.dir();
    const Eigen::Vector3d& normal_outer = intersect_outer->normal;
    const Eigen::Vector3d& normal_inner = intersect_inner->normal;

    std::cout << "point: " << point_outer.transpose()
              << ", normal: " << intersect_outer->normal.transpose() << std::endl;
    std::cout << "point: " << point_inner.transpose()
              << ", normal: " << intersect_inner->normal.transpose() << std::endl;

    file << point_outer(0) << " " << point_outer(1) << " " << point_outer(2) << " "
         << normal_outer(0) << " " << normal_outer(1) << " " << normal_outer(2) << std::endl;
    file << point_inner(0) << " " << point_inner(1) << " " << point_inner(2) << " "
         << normal_inner(0) << " " << normal_inner(1) << " " << normal_inner(2) << std::endl;
  };

  for (size_t i = 0; i < kNumGrid; i++) {
    const double x = 2. * kRadius / (kNumGrid - 1) * i - kRadius;

    for (size_t j = 0; j < kNumGrid; j++) {
      const double y = 2. * kRadius / (kNumGrid - 1) * j - kRadius;

      for (size_t k = 0; k < 2; k++) {
        const double z = 2. * kRadius * k - kRadius;

        ProjectFrom(Eigen::Vector3d(x, y, z));
        ProjectFrom(Eigen::Vector3d(z, x, y));
        ProjectFrom(Eigen::Vector3d(y, z, x));
      }
    }
  }

  file.close();

  return 0;
}
