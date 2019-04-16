/**
 * mod.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

#[cfg(feature = "dim2")]
mod math2d;

#[cfg(feature = "dim3")]
mod math3d;

#[cfg(feature = "dim2")]
pub use math2d::*;

#[cfg(feature = "dim3")]
pub use math3d::*;
