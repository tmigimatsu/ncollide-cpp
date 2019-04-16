/**
 * point.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: April 16, 2019
 * Authors: Toki Migimatsu
 */

use crate::nc;
use crate::math;

#[repr(C)]
pub struct CPointProjection {
    pub is_inside: bool,
    pub point: [f64; nc::math::DIM]
}

pub fn contains_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                      m: Option<&math::CIsometry>,
                      pt: Option<&[f64; nc::math::DIM]>) -> bool {
    use nc::query::PointQuery;

    let shape = shape.unwrap().as_ref();
    let m = math::isometry_from_raw(m.unwrap());
    let pt = nc::math::Point::from(nc::math::Vector::from_column_slice(pt.unwrap()));
    shape.contains_point(&m, &pt)
}

pub fn distance_to_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                         m: Option<&math::CIsometry>,
                         pt: Option<&[f64; nc::math::DIM]>,
                         solid: bool) -> f64 {
    use nc::query::PointQuery;

    let shape = shape.unwrap().as_ref();
    let m = math::isometry_from_raw(m.unwrap());
    let pt = nc::math::Point::from(nc::math::Vector::from_column_slice(pt.unwrap()));
    shape.distance_to_point(&m, &pt, solid)
}

pub fn project_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                     m: Option<&math::CIsometry>,
                     pt: Option<&[f64; nc::math::DIM]>,
                     solid: bool,
                     out_projection: Option<&mut CPointProjection>) {
    use nc::query::PointQuery;

    let shape = shape.unwrap().as_ref();
    let m = math::isometry_from_raw(m.unwrap());
    let pt = nc::math::Point::from(nc::math::Vector::from_column_slice(pt.unwrap()));
    let projection = shape.project_point(&m, &pt, solid);
    match out_projection {
        Some(out_projection) => {
            out_projection.is_inside = projection.is_inside;
            out_projection.point.copy_from_slice(projection.point.coords.data.as_slice());
        },
        None => {}
    };
}
