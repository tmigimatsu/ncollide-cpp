/**
 * mod.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

mod ball;
mod capsule;
mod compound;
mod cuboid;

// #[cfg(feature = "dim3")]
// mod cylinder;

#[cfg(feature = "dim2")]
mod convex_polygon;

#[cfg(feature = "dim3")]
mod convex_hull;

#[cfg(feature = "dim3")]
mod rounded_cuboid;  // TODO: Implement for 2d

#[cfg(feature = "dim2")]
mod shape2d;

#[cfg(feature = "dim3")]
mod shape3d;

#[cfg(feature = "dim3")]
mod trimesh;
