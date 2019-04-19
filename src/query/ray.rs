/**
 * ray.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: April 16, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;

use crate::nc;
use crate::math;

#[repr(C)]
pub struct CRayIntersection {
    pub toi: f64,
    pub normal: [f64; nc::math::DIM],
}

pub fn ray_new(ptr_origin: Option<&[f64; nc::math::DIM]>,
               ptr_dir: Option<&[f64; nc::math::DIM]>) -> *mut nc::query::Ray<f64> {
    let origin = nc::math::Point::from_slice(ptr_origin.unwrap());
    let dir = nc::math::Vector::from_column_slice(ptr_dir.unwrap());
    let ray = nc::query::Ray::new(origin, dir);
    Box::into_raw(Box::new(ray))
}

pub fn ray_origin(ray: Option<&nc::query::Ray<f64>>) -> *const f64 {
    use na::storage::Storage;

    let ray = ray.unwrap();
    ray.origin.coords.data.ptr()
}

pub fn ray_dir(ray: Option<&nc::query::Ray<f64>>) -> *const f64 {
    use na::storage::Storage;

    let ray = ray.unwrap();
    ray.dir.data.ptr()
}

pub fn toi_with_ray(shape: Option<&nc::shape::ShapeHandle<f64>>,
                    m: Option<&math::CIsometry>,
                    ray: Option<&nc::query::Ray<f64>>,
                    solid: bool,
                    out_toi: Option<&mut f64>) -> bool {
    use nc::query::RayCast;

    let shape = shape.unwrap().as_ref();
    let m = math::isometry_from_raw(m.unwrap());
    let ray = ray.unwrap();

    let toi = shape.toi_with_ray(&m, &ray, solid);
    match toi {
        Some(toi) => {
            match out_toi {
                Some(out_toi) => { *out_toi = toi; },
                None => {}
            };
            true
        },
        None => false
    }
}

pub fn toi_and_normal_with_ray(shape: Option<&nc::shape::ShapeHandle<f64>>,
                               m: Option<&math::CIsometry>,
                               ray: Option<&nc::query::Ray<f64>>,
                               solid: bool,
                               out_intersect: Option<&mut CRayIntersection>) -> bool {
    use nc::query::RayCast;

    let shape = shape.unwrap().as_ref();
    let m = math::isometry_from_raw(m.unwrap());
    let ray = ray.unwrap();

    let intersect = shape.toi_and_normal_with_ray(&m, &ray, solid);
    match intersect {
        Some(intersect) => {
            match out_intersect {
                Some(out_intersect) => {
                    out_intersect.toi = intersect.toi;
                    out_intersect.normal.copy_from_slice(intersect.normal.data.as_slice()); 
                },
                None => {}
            };
            true
        },
        None => false
    }
}
