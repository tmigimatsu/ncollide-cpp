/**
 * shape3d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

use crate::nc;
use crate::math;
use super::{ball, capsule, compound, convex_hull, cuboid, rounded_cuboid, trimesh};

/**
 * Ball
 */

#[no_mangle]
extern fn ncollide3d_shape_ball_new(radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    ball::ball_new(radius)
}

#[no_mangle]
extern fn ncollide3d_shape_ball_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    ball::ball_radius(shape)
}

/**
 * Capsule
 */

#[no_mangle]
extern fn ncollide3d_shape_capsule_new(half_height: f64, radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    capsule::capsule_new(half_height, radius)
}

#[no_mangle]
extern fn ncollide3d_shape_capsule_half_height(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    capsule::capsule_half_height(shape)
}

#[no_mangle]
extern fn ncollide3d_shape_capsule_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    capsule::capsule_radius(shape)
}

/**
 * Compound
 */

#[no_mangle]
extern fn ncollide3d_shape_compound_new(ptr_transforms: *const math::CIsometry,
                                        ptr_shapes: *const Option<&nc::shape::ShapeHandle<f64>>,
                                        n: usize) -> *mut nc::shape::ShapeHandle<f64> {
    compound::compound_new(ptr_transforms, ptr_shapes, n)
}

/**
 * Convex hull
 */

#[no_mangle]
extern fn ncollide3d_shape_convex_hull_try_from_points(ptr_points: *const [f64; nc::math::DIM],
                                                       npoints: usize) -> *mut nc::shape::ShapeHandle<f64> {
    convex_hull::convex_hull_new(ptr_points, npoints)
}

#[no_mangle]
extern fn ncollide3d_shape_convex_hull_num_points(shape: Option<&nc::shape::ShapeHandle<f64>>) -> usize {
    convex_hull::convex_hull_num_points(shape)
}

#[no_mangle]
extern fn ncollide3d_shape_convex_hull_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                                         i: usize) -> *const f64 {
    convex_hull::convex_hull_point(shape, i)
}

/**
 * Cuboid
 */

#[no_mangle]
extern fn ncollide3d_shape_cuboid_new(x: f64, y: f64, z: f64) -> *mut nc::shape::ShapeHandle<f64> {
    cuboid::cuboid_new(x, y, z)
}

#[no_mangle]
extern fn ncollide3d_shape_cuboid_half_extents(shape: Option<&nc::shape::ShapeHandle<f64>>) -> *const f64 {
    cuboid::cuboid_half_extents(shape)
}

#[no_mangle]
extern fn ncollide3d_shape_cuboid_to_trimesh(shape: Option<&nc::shape::ShapeHandle<f64>>)
        -> *const nc::shape::ShapeHandle<f64> {
    cuboid::cuboid_to_trimesh(shape)
}

/**
 * Cylinder
 */

// #[no_mangle]
// extern fn ncollide3d_shape_cylinder_new(half_height: f64, radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
//     cylinder::cylinder_new(half_height, radius)
// }

// #[no_mangle]
// extern fn ncollide3d_shape_cylinder_half_height(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
//     cylinder::cylinder_half_height(shape)
// }

// #[no_mangle]
// extern fn ncollide3d_shape_cylinder_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
//     cylinder::cylinder_radius(shape)
// }

/**
 * Rounded cuboid
 */

#[no_mangle]
extern fn ncollide3d_shape_rounded_cuboid_new(x: f64, y: f64, z: f64, radius: f64)
        -> *mut nc::shape::ShapeHandle<f64> {
    rounded_cuboid::rounded_cuboid_new(x, y, z, radius)
}

#[no_mangle]
extern fn ncollide3d_shape_rounded_cuboid_half_extents(shape: Option<&nc::shape::ShapeHandle<f64>>)
        -> *const f64 {
    rounded_cuboid::rounded_cuboid_half_extents(shape)
}

#[no_mangle]
extern fn ncollide3d_shape_rounded_cuboid_radius(shape: Option<&nc::shape::ShapeHandle<f64>>)
        -> f64 {
    rounded_cuboid::rounded_cuboid_radius(shape)
}

/**
 * Trimesh
 */

#[no_mangle]
extern fn ncollide3d_shape_trimesh_new(ptr_points: *const [f64; nc::math::DIM],
                                       npoints: usize,
                                       ptr_indices: *const [usize; nc::math::DIM],
                                       nfaces: usize) -> *mut nc::shape::ShapeHandle<f64> {
    trimesh::trimesh_new(ptr_points, npoints, ptr_indices, nfaces)
}

#[no_mangle]
extern fn ncollide3d_shape_trimesh_num_points(shape: Option<&nc::shape::ShapeHandle<f64>>) -> usize {
    trimesh::trimesh_num_points(shape)
}

#[no_mangle]
extern fn ncollide3d_shape_trimesh_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                                         i: usize) -> *const f64 {
    trimesh::trimesh_point(shape, i)
}

#[no_mangle]
extern fn ncollide3d_shape_trimesh_file(filename: &str) -> *mut nc::shape::ShapeHandle<f64> {
    trimesh::trimesh_file(filename)
}

/**
 * Shape delete
 */

#[no_mangle]
unsafe extern fn ncollide3d_shape_delete(ptr: *mut nc::shape::ShapeHandle<f64>) {
    Box::from_raw(ptr);
}
