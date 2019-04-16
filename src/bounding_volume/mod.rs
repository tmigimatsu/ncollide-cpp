/**
 * mod.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

mod aabb;

#[cfg(feature = "dim2")]
mod bounding_volume2d;

#[cfg(feature = "dim3")]
mod bounding_volume3d;
