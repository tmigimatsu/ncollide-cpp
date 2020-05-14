/**
 * query3d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

use crate::math;
use super::{ray, pairwise, point};

/**
 * Point queries
 */

#[no_mangle]
extern fn ncollide3d_query_contains_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                                          m: Option<&math::CIsometry>,
                                          pt: Option<&[f64; nc::math::DIM]>) -> bool {
    point::contains_point(shape, m, pt)
}

#[no_mangle]
extern fn ncollide3d_query_distance_to_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                                             m: Option<&math::CIsometry>,
                                             pt: Option<&[f64; nc::math::DIM]>,
                                             solid: bool) -> f64 {
    point::distance_to_point(shape, m, pt, solid)
}

#[no_mangle]
extern fn ncollide3d_query_project_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                                         m: Option<&math::CIsometry>,
                                         pt: Option<&[f64; nc::math::DIM]>,
                                         solid: bool,
                                         out_projection: Option<&mut point::CPointProjection>) {
    point::project_point(shape, m, pt, solid, out_projection)
}

/**
 * Ray casting
 */

#[no_mangle]
extern fn ncollide3d_query_ray_new(ptr_origin: Option<&[f64; nc::math::DIM]>,
                                   ptr_dir: Option<&[f64; nc::math::DIM]>) -> *mut nc::query::Ray<f64> {
    ray::ray_new(ptr_origin, ptr_dir)
}

#[no_mangle]
unsafe extern fn ncollide3d_query_ray_delete(ptr: *mut nc::query::Ray<f64>) {
    Box::from_raw(ptr);
}

#[no_mangle]
extern fn ncollide3d_query_ray_origin(ray: Option<&nc::query::Ray<f64>>) -> *const f64 {
    ray::ray_origin(ray)
}

#[no_mangle]
extern fn ncollide3d_query_ray_dir(ray: Option<&nc::query::Ray<f64>>) -> *const f64 {
    ray::ray_dir(ray)
}

#[no_mangle]
extern fn ncollide3d_query_toi_with_ray(shape: Option<&nc::shape::ShapeHandle<f64>>,
                                        m: Option<&math::CIsometry>,
                                        ray: Option<&nc::query::Ray<f64>>,
                                        max_toi: f64,
                                        solid: bool,
                                        out_toi: Option<&mut f64>) -> bool {
    ray::toi_with_ray(shape, m, ray, max_toi, solid, out_toi)
}

#[no_mangle]
extern fn ncollide3d_query_toi_and_normal_with_ray(shape: Option<&nc::shape::ShapeHandle<f64>>,
                                                   m: Option<&math::CIsometry>,
                                                   ray: Option<&nc::query::Ray<f64>>,
                                                   max_toi: f64,
                                                   solid: bool,
                                                   out_intersect: Option<&mut ray::CRayIntersection>) -> bool {
    ray::toi_and_normal_with_ray(shape, m, ray, max_toi, solid, out_intersect)
}

/**
 * Pairwise queries
 */

#[no_mangle]
extern fn ncollide3d_query_closest_points(m1: Option<&math::CIsometry>,
                                          g1: Option<&nc::shape::ShapeHandle<f64>>,
                                          m2: Option<&math::CIsometry>,
                                          g2: Option<&nc::shape::ShapeHandle<f64>>,
                                          max_dist: f64,
                                          out_p1: Option<&mut [f64; nc::math::DIM]>,
                                          out_p2: Option<&mut [f64; nc::math::DIM]>) -> pairwise::CClosestPoints {
    pairwise::closest_points(m1, g1, m2, g2, max_dist, out_p1, out_p2)
}

#[no_mangle]
extern fn ncollide3d_query_contact(m1: Option<&math::CIsometry>,
                                   g1: Option<&nc::shape::ShapeHandle<f64>>,
                                   m2: Option<&math::CIsometry>,
                                   g2: Option<&nc::shape::ShapeHandle<f64>>,
                                   prediction: f64,
                                   out_contact: Option<&mut pairwise::CContact>) -> bool {
    pairwise::contact(m1, g1, m2, g2, prediction, out_contact)
}

#[no_mangle]
extern fn ncollide3d_query_distance(m1: Option<&math::CIsometry>,
                                    g1: Option<&nc::shape::ShapeHandle<f64>>,
                                    m2: Option<&math::CIsometry>,
                                    g2: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    pairwise::distance(m1, g1, m2, g2)
}

#[no_mangle]
extern fn ncollide3d_query_proximity(m1: Option<&math::CIsometry>,
                                     g1: Option<&nc::shape::ShapeHandle<f64>>,
                                     m2: Option<&math::CIsometry>,
                                     g2: Option<&nc::shape::ShapeHandle<f64>>,
                                     margin: f64) -> pairwise::CProximity {
    pairwise::proximity(m1, g1, m2, g2, margin)
}

#[no_mangle]
extern fn ncollide3d_query_time_of_impact(m1: Option<&math::CIsometry>,
                                          v1: Option<&[f64; nc::math::DIM]>,
                                          g1: Option<&nc::shape::ShapeHandle<f64>>,
                                          m2: Option<&math::CIsometry>,
                                          v2: Option<&[f64; nc::math::DIM]>,
                                          g2: Option<&nc::shape::ShapeHandle<f64>>,
                                          max_toi: f64,
                                          target_distance: f64,
                                          out_time: Option<&mut pairwise::CTOI>) -> bool {
    pairwise::time_of_impact(m1, v1, g1, m2, v2, g2, max_toi, target_distance, out_time)
}
