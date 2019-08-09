/**
 * bounding_volume2d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 20, 2019
 * Authors: Toki Migimatsu
 */

use crate::nc;
use crate::math;
use super::aabb;
use super::bounding_sphere;

#[no_mangle]
extern fn ncollide2d_bounding_volume_aabb(g: Option<&nc::shape::ShapeHandle<f64>>,
                                          m: Option<&math::CIsometry>)
        -> *mut nc::bounding_volume::AABB<f64> {
    aabb::aabb(g, m)
}

#[no_mangle]
unsafe extern fn ncollide2d_bounding_volume_aabb_delete(ptr: *mut nc::bounding_volume::AABB<f64>) {
    Box::from_raw(ptr);
}

#[no_mangle]
extern fn ncollide2d_bounding_volume_aabb_maxs(aabb: Option<&nc::bounding_volume::AABB<f64>>) -> *const f64 {
    aabb::aabb_maxs(aabb)
}

#[no_mangle]
extern fn ncollide2d_bounding_volume_aabb_mins(aabb: Option<&nc::bounding_volume::AABB<f64>>) -> *const f64 {
    aabb::aabb_mins(aabb)
}

#[no_mangle]
extern fn ncollide2d_bounding_volume_bounding_sphere(g: Option<&nc::shape::ShapeHandle<f64>>,
                                                     m: Option<&math::CIsometry>)
        -> *mut nc::bounding_volume::BoundingSphere<f64> {
    bounding_sphere::bounding_sphere(g, m)
}

#[no_mangle]
extern fn ncollide2d_bounding_volume_bounding_sphere_radius(ptr: Option<&nc::bounding_volume::BoundingSphere<f64>>) -> f64 {
    bounding_sphere::bounding_sphere_radius(ptr)
}

#[no_mangle]
unsafe extern fn ncollide2d_bounding_volume_bounding_sphere_delete(ptr: *mut nc::bounding_volume::BoundingSphere<f64>) {
    Box::from_raw(ptr);
}
