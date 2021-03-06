/**
 * math3d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

use crate::nc;

#[repr(C)]
pub struct CIsometry {
    translation: [f64; nc::math::DIM],
    rotation: [f64; nc::math::DIM]
}

pub fn isometry_from_raw(isometry: &CIsometry) -> nc::math::Isometry<f64> {
    let translation = nc::math::Vector::from_column_slice(&isometry.translation);
    let rotation = nc::math::Orientation::from_column_slice(&isometry.rotation);
    nc::math::Isometry::new(translation, rotation)
}
