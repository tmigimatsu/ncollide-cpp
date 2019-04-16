/**
 * ball.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

use crate::nc;

pub fn ball_new(radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let ball = nc::shape::Ball::new(radius);
    let handle = nc::shape::ShapeHandle::new(ball);
    Box::into_raw(Box::new(handle))
}

pub fn ball_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let maybe_ball = shape.unwrap().as_shape::<nc::shape::Ball<f64>>();
    match maybe_ball {
        Some(ref ball) => { ball.radius() },
        None => { 0. }
    }
}
