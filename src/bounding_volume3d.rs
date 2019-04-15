/**
 * bounding_volume3d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 20, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;
extern crate ncollide3d as nc;

use super::math3d::*;

#[no_mangle]
pub extern fn ncollide3d_bounding_volume_aabb(g: Option<&nc::shape::ShapeHandle<f64>>,
                                              m: Option<&ncollide3d_math_isometry_t>)
        -> *mut nc::bounding_volume::AABB<f64> {
    let m = isometry_from_raw(m.unwrap());
    let aabb = g.unwrap().as_ref().aabb(&m);
    Box::into_raw(Box::new(aabb))
}

#[no_mangle]
pub unsafe extern fn ncollide3d_bounding_volume_aabb_delete(ptr: *mut nc::bounding_volume::AABB<f64>) {
    Box::from_raw(ptr);
}

#[no_mangle]
pub extern fn ncollide3d_bounding_volume_aabb_maxs(aabb: Option<&nc::bounding_volume::AABB<f64>>)
        -> *const f64 {
    use na::storage::Storage;

    let aabb = aabb.unwrap();
    aabb.maxs().coords.data.ptr()
}

#[no_mangle]
pub extern fn ncollide3d_bounding_volume_aabb_mins(aabb: Option<&nc::bounding_volume::AABB<f64>>)
        -> *const f64 {
    use na::storage::Storage;

    let aabb = aabb.unwrap();
    aabb.mins().coords.data.ptr()
}
