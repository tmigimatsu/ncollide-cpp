/**
 * convex_hull.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: July 12, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;

use crate::nc;

pub fn convex_hull_new(ptr_points: *const [f64; nc::math::DIM],
                       npoints: usize) -> *mut nc::shape::ShapeHandle<f64> {

    let points = unsafe { std::slice::from_raw_parts(ptr_points, npoints) };
    let points: Vec<_> = points.iter().map(|x| nc::math::Point::<f64>::from_slice(x)).collect();

    let maybe_trimesh = nc::shape::ConvexHull::try_from_points(&points);
    match maybe_trimesh {
        Some(trimesh) => {
            let handle = nc::shape::ShapeHandle::new(trimesh);
            Box::into_raw(Box::new(handle))
        }
        None => {
            let trimesh = nc::shape::TriMesh::new(Vec::new(), Vec::new(), Option::None);
            Box::into_raw(Box::new(nc::shape::ShapeHandle::new(trimesh)))
        }
    }
}

pub fn convex_hull_num_points(shape: Option<&nc::shape::ShapeHandle<f64>>) -> usize {
    let maybe_convex = shape.unwrap().as_shape::<nc::shape::ConvexHull<f64>>();
    match maybe_convex {
        Some(ref convex) =>  { convex.points().len() }
        None => { 0 }
    }
}

pub fn convex_hull_point(shape: Option<&nc::shape::ShapeHandle<f64>>, i: usize) -> *const f64 {
    use na::storage::Storage;

    let maybe_convex = shape.unwrap().as_shape::<nc::shape::ConvexHull<f64>>();
    match maybe_convex {
        Some(ref convex) =>  { convex.points()[i].coords.data.ptr() }
        None => { std::ptr::null() }
    }
}

