/**
 * lib.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 12, 2019
 * Authors: Toki Migimatsu
 */

#[cfg(feature = "dim2")]
extern crate ncollide2d as nc;

#[cfg(feature = "dim3")]
extern crate ncollide3d as nc;

mod bounding_volume;
mod math;
mod query;
mod shape;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
