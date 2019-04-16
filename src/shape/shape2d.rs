/**
 * shape2d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

use crate::nc;
use crate::math;
use super::{ball, capsule, compound, cuboid, convex_polygon};

/**
 * Ball
 */

#[no_mangle]
extern fn ncollide2d_shape_ball_new(radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    ball::ball_new(radius)
}

#[no_mangle]
extern fn ncollide2d_shape_ball_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    ball::ball_radius(shape)
}

/**
 * Capsule
 */

#[no_mangle]
extern fn ncollide2d_shape_capsule_new(half_height: f64, radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    capsule::capsule_new(half_height, radius)
}

#[no_mangle]
extern fn ncollide2d_shape_capsule_half_height(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    capsule::capsule_half_height(shape)
}

#[no_mangle]
extern fn ncollide2d_shape_capsule_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    capsule::capsule_radius(shape)
}

/**
 * Compound
 */

#[no_mangle]
extern fn ncollide2d_shape_compound_new(ptr_transforms: *const math::CIsometry,
                                        ptr_shapes: *const Option<&nc::shape::ShapeHandle<f64>>,
                                        n: usize) -> *mut nc::shape::ShapeHandle<f64> {
    compound::compound_new(ptr_transforms, ptr_shapes, n)
}

/**
 * Cuboid
 */

#[no_mangle]
extern fn ncollide2d_shape_cuboid_new(x: f64, y: f64) -> *mut nc::shape::ShapeHandle<f64> {
    cuboid::cuboid_new(x, y)
}

#[no_mangle]
extern fn ncollide2d_shape_cuboid_half_extents(shape: Option<&nc::shape::ShapeHandle<f64>>) -> *const f64 {
    cuboid::cuboid_half_extents(shape)
}

/**
 * Convex polygon
 */

#[no_mangle]
extern fn ncollide2d_shape_convex_polygon_try_from_points(ptr_points: *const [f64; nc::math::DIM],
                                                          n: usize) -> *mut nc::shape::ShapeHandle<f64> {
    convex_polygon::convex_polygon_try_from_points(ptr_points, n)
}

/**
 * Shape delete
 */

#[no_mangle]
unsafe extern fn ncollide2d_shape_delete(ptr: *mut nc::shape::ShapeHandle<f64>) {
    Box::from_raw(ptr);
}
