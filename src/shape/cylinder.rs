/**
 * cylinder.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 20, 2019
 * Authors: Toki Migimatsu
 */

use crate::nc;

pub fn cylinder_new(half_height: f64, radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let cylinder = nc::shape::Cylinder::new(half_height, radius);
    let handle = nc::shape::ShapeHandle::new(cylinder);
    Box::into_raw(Box::new(handle))
}

pub fn cylinder_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let maybe_cylinder = shape.unwrap().as_shape::<nc::shape::Cylinder<f64>>();
    match maybe_cylinder {
        Some(ref cylinder) => { cylinder.radius() },
        None => { 0. }
    }
}

pub fn cylinder_half_height(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let maybe_cylinder = shape.unwrap().as_shape::<nc::shape::Cylinder<f64>>();
    match maybe_cylinder {
        Some(ref cylinder) => { cylinder.half_height() },
        None => { 0. }
    }
}
