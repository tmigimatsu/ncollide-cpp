/**
 * mod.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: July 8, 2019
 * Authors: Toki Migimatsu
 */

use super::convex_hull;

#[no_mangle]
extern fn ncollide3d_transformation_convex_hull(ptr_points: *const [f64; nc::math::DIM],
                                                npoints: usize)
        -> *mut nc::shape::ShapeHandle<f64> {
    convex_hull::convex_hull(ptr_points, npoints)
}

