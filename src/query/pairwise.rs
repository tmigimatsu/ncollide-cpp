/**
 * pairwise.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: April 16, 2019
 * Authors: Toki Migimatsu
 */

use crate::nc;
use crate::math;

#[repr(C)]
pub enum CClosestPoints {
    Intersecting,
    WithinMargin,
    Disjoint
}

#[repr(C)]
pub struct CContact {
    pub world1: [f64; nc::math::DIM],
    pub world2: [f64; nc::math::DIM],
    pub normal: [f64; nc::math::DIM],
    pub depth: f64
}

#[repr(C)]
pub enum CProximity {
    Intersecting,
    WithinMargin,
    Disjoint
}

pub fn closest_points(m1: Option<&math::CIsometry>,
                      g1: Option<&nc::shape::ShapeHandle<f64>>,
                      m2: Option<&math::CIsometry>,
                      g2: Option<&nc::shape::ShapeHandle<f64>>,
                      max_dist: f64,
                      out_p1: Option<&mut [f64; nc::math::DIM]>,
                      out_p2: Option<&mut [f64; nc::math::DIM]>) -> CClosestPoints {
    let m1 = math::isometry_from_raw(m1.unwrap());
    let m2 = math::isometry_from_raw(m2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();

    let result = nc::query::closest_points(&m1, g1, &m2, g2, max_dist);
    match result {
        nc::query::ClosestPoints::Intersecting => CClosestPoints::Intersecting,
        nc::query::ClosestPoints::WithinMargin(ref p1, ref p2) => {
            match out_p1 {
                Some(out_p1) => {
                    out_p1.copy_from_slice(p1.coords.data.as_slice());
                },
                None => {}
            };
            match out_p2 {
                Some(out_p2) => {
                    out_p2.copy_from_slice(p2.coords.data.as_slice());
                },
                None => {}
            };
            CClosestPoints::WithinMargin
        },
        nc::query::ClosestPoints::Disjoint => CClosestPoints::Disjoint
    }
}

pub fn contact(m1: Option<&math::CIsometry>,
               g1: Option<&nc::shape::ShapeHandle<f64>>,
               m2: Option<&math::CIsometry>,
               g2: Option<&nc::shape::ShapeHandle<f64>>,
               prediction: f64,
               out_contact: Option<&mut CContact>) -> bool {
    let m1 = math::isometry_from_raw(m1.unwrap());
    let m2 = math::isometry_from_raw(m2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();

    let result = nc::query::contact(&m1, g1, &m2, g2, prediction);
    match result {
        Some(ref contact) => {
            match out_contact {
                Some(out_contact) => {
                    out_contact.world1.copy_from_slice(contact.world1.coords.data.as_slice());
                    out_contact.world2.copy_from_slice(contact.world2.coords.data.as_slice());
                    out_contact.normal.copy_from_slice(contact.normal.data.as_slice());
                    out_contact.depth = contact.depth;
                },
                None => {}
            };
            true
        },
        None => false
    }
}

pub fn distance(m1: Option<&math::CIsometry>,
                g1: Option<&nc::shape::ShapeHandle<f64>>,
                m2: Option<&math::CIsometry>,
                g2: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let m1 = math::isometry_from_raw(m1.unwrap());
    let m2 = math::isometry_from_raw(m2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();
    nc::query::distance(&m1, g1, &m2, g2)
}

pub fn proximity(m1: Option<&math::CIsometry>,
                 g1: Option<&nc::shape::ShapeHandle<f64>>,
                 m2: Option<&math::CIsometry>,
                 g2: Option<&nc::shape::ShapeHandle<f64>>,
                 margin: f64) -> CProximity {
    let m1 = math::isometry_from_raw(m1.unwrap());
    let m2 = math::isometry_from_raw(m2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();

    let result = nc::query::proximity(&m1, g1, &m2, g2, margin);
    match result {
        nc::query::Proximity::Intersecting => CProximity::Intersecting,
        nc::query::Proximity::WithinMargin => CProximity::WithinMargin,
        nc::query::Proximity::Disjoint => CProximity::Disjoint,
    }
}

pub extern fn time_of_impact(m1: Option<&math::CIsometry>,
                             v1: Option<&[f64; nc::math::DIM]>,
                             g1: Option<&nc::shape::ShapeHandle<f64>>,
                             m2: Option<&math::CIsometry>,
                             v2: Option<&[f64; nc::math::DIM]>,
                             g2: Option<&nc::shape::ShapeHandle<f64>>,
                             out_time: Option<&mut f64>) -> bool {
    let m1 = math::isometry_from_raw(m1.unwrap());
    let m2 = math::isometry_from_raw(m2.unwrap());
    let v1 = nc::math::Vector::from_column_slice(v1.unwrap());
    let v2 = nc::math::Vector::from_column_slice(v2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();

    let result = nc::query::time_of_impact(&m1, &v1, g1, &m2, &v2, g2);
    match result {
        Some(time) => {
            match out_time {
                Some(out_time) => { *out_time = time; },
                None => {}
            };
            true
        },
        None => false
    }
}
