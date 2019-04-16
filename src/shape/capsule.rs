/**
 * capsule.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

use crate::nc;

pub fn capsule_new(half_height: f64, radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let capsule = nc::shape::Capsule::new(half_height, radius);
    let handle = nc::shape::ShapeHandle::new(capsule);
    Box::into_raw(Box::new(handle))
}

pub fn capsule_half_height(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let maybe_capsule = shape.unwrap().as_shape::<nc::shape::Capsule<f64>>();
    match maybe_capsule {
        Some(ref capsule) => { capsule.half_height() },
        None => { 0. }
    }
}

pub fn capsule_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let maybe_capsule = shape.unwrap().as_shape::<nc::shape::Capsule<f64>>();
    match maybe_capsule {
        Some(ref capsule) => { capsule.radius() },
        None => { 0. }
    }
}
