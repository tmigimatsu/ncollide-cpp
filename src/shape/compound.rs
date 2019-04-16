/**
 * compound.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;

use crate::nc;
use crate::math;

pub fn compound_new(ptr_transforms: *const math::CIsometry,
                    ptr_shapes: *const Option<&nc::shape::ShapeHandle<f64>>,
                    n: usize) -> *mut nc::shape::ShapeHandle<f64> {
    use nc::math::Isometry;
    use nc::shape::ShapeHandle;

    let transforms = unsafe { std::slice::from_raw_parts(ptr_transforms, n) };
    let transforms = transforms.iter().map(|x| math::isometry_from_raw(x));

    let shapes = unsafe { std::slice::from_raw_parts(ptr_shapes, n) };
    let shapes = shapes.iter().map(|x| x.unwrap().clone());

    let transforms_shapes: Vec<(Isometry<f64>, ShapeHandle<f64>)> = transforms.zip(shapes).collect();

    let compound = nc::shape::Compound::new(transforms_shapes);
    let handle = ShapeHandle::new(compound);
    Box::into_raw(Box::new(handle))
}
