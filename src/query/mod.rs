/**
 * mod.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

mod pairwise;
mod point;
mod ray;

#[cfg(feature = "dim2")]
mod query2d;

#[cfg(feature = "dim3")]
mod query3d;
