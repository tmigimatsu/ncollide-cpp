/**
 * aabb.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 20, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;

use crate::nc;
use crate::math;

pub fn aabb(g: Option<&nc::shape::ShapeHandle<f64>>,
            m: Option<&math::CIsometry>) -> *mut nc::bounding_volume::AABB<f64> {
    let m = math::isometry_from_raw(m.unwrap());
    let aabb = g.unwrap().as_ref().aabb(&m);
    Box::into_raw(Box::new(aabb))
}

pub fn aabb_maxs(aabb: Option<&nc::bounding_volume::AABB<f64>>) -> *const f64 {
    use na::storage::Storage;

    let aabb = aabb.unwrap();
    aabb.maxs().coords.data.ptr()
}

pub fn aabb_mins(aabb: Option<&nc::bounding_volume::AABB<f64>>) -> *const f64 {
    use na::storage::Storage;

    let aabb = aabb.unwrap();
    aabb.mins().coords.data.ptr()
}
