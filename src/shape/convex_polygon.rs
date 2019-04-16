/**
 * convex_polygon.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;

use crate::nc;

pub fn convex_polygon_try_from_points(ptr_points: *const [f64; nc::math::DIM], n: usize)
        -> *mut nc::shape::ShapeHandle<f64> {
    use nc::math::Point;

    let points = unsafe { std::slice::from_raw_parts(ptr_points, n) };
    let points: Vec<_> = points.iter().map(|x| Point::<f64>::from_slice(x)).collect();

    let polygon = nc::shape::ConvexPolygon::try_from_points(&points).unwrap();
    let handle = nc::shape::ShapeHandle::new(polygon);
    Box::into_raw(Box::new(handle))
}
