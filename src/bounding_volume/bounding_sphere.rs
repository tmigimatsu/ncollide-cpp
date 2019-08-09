/**
 * bounding_sphere.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 20, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;

use crate::nc;
use crate::math;

pub fn bounding_sphere(g: Option<&nc::shape::ShapeHandle<f64>>,
                       m: Option<&math::CIsometry>)
        -> *mut nc::bounding_volume::BoundingSphere<f64> {
    let m = math::isometry_from_raw(m.unwrap());
    let sphere = g.unwrap().as_ref().bounding_sphere(&m);
    Box::into_raw(Box::new(sphere))
}

pub fn bounding_sphere_radius(sphere: Option<&nc::bounding_volume::BoundingSphere<f64>>) -> f64 {
    sphere.unwrap().radius()
}

