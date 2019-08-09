/**
 * cuboid.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;

use crate::nc;

#[cfg(feature = "dim2")]
pub fn cuboid_new(x: f64, y: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let cuboid = nc::shape::Cuboid::new(nc::math::Vector::new(x, y));
    let handle = nc::shape::ShapeHandle::new(cuboid);
    Box::into_raw(Box::new(handle))
}

#[cfg(feature = "dim3")]
pub fn cuboid_new(x: f64, y: f64, z: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let cuboid = nc::shape::Cuboid::new(nc::math::Vector::new(x, y, z));
    let handle = nc::shape::ShapeHandle::new(cuboid);
    Box::into_raw(Box::new(handle))
}

pub fn cuboid_half_extents(shape: Option<&nc::shape::ShapeHandle<f64>>) -> *const f64 {
    use na::storage::Storage;

    let maybe_cuboid = shape.unwrap().as_shape::<nc::shape::Cuboid<f64>>();
    match maybe_cuboid {
        Some(ref cuboid) => { cuboid.half_extents().data.ptr() },
        None => { std::ptr::null() }
    }
}

#[cfg(feature = "dim3")]
pub fn cuboid_to_trimesh(shape: Option<&nc::shape::ShapeHandle<f64>>)
        -> *const nc::shape::ShapeHandle<f64> {
    use crate::nc::transformation::ToTriMesh;
    let maybe_cuboid = shape.unwrap().as_shape::<nc::shape::Cuboid<f64>>();
    match maybe_cuboid {
        Some(ref cuboid) => {
            let trimesh = nc::shape::TriMesh::<f64>::from(cuboid.to_trimesh(()));
            let handle = nc::shape::ShapeHandle::new(trimesh);
            Box::into_raw(Box::new(handle))
        },
        None => { std::ptr::null() }
    }
}
