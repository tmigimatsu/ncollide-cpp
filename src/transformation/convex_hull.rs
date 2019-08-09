/**
 * convex_hull.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: July 8, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;

use crate::nc;

pub fn convex_hull(ptr_points: *const [f64; nc::math::DIM],
                   npoints: usize) -> *mut nc::shape::ShapeHandle<f64> {

    let points = unsafe { std::slice::from_raw_parts(ptr_points, npoints) };
    let points: Vec<_> = points.iter().map(|x| nc::math::Point::<f64>::from_slice(x)).collect();

    let trimesh = nc::shape::TriMesh::from(nc::transformation::convex_hull(&points));
    let handle = nc::shape::ShapeHandle::new(trimesh);
    Box::into_raw(Box::new(handle))
}

