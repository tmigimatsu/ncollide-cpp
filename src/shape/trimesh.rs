/**
 * trimesh.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;

use crate::nc;

pub fn trimesh_new(ptr_points: *const [f64; nc::math::DIM],
                   npoints: usize,
                   ptr_indices: *const [usize; nc::math::DIM],
                   nfaces: usize) -> *mut nc::shape::ShapeHandle<f64> {
    use nc::math::Point;
    use na::geometry::Point3;

    let points = unsafe { std::slice::from_raw_parts(ptr_points, npoints) };
    let points: Vec<_> = points.iter().map(|x| Point::<f64>::from_slice(x)).collect();

    let indices = unsafe { std::slice::from_raw_parts(ptr_indices, nfaces) };
    let indices: Vec<_> = indices.iter().map(|x| Point3::<usize>::from_slice(x)).collect();

    let trimesh = nc::shape::TriMesh::new(points, indices, None);
    let handle = nc::shape::ShapeHandle::new(trimesh);
    Box::into_raw(Box::new(handle))
}

#[derive(Debug)]
struct ParseVectorError;
impl std::fmt::Display for ParseVectorError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result { write!(f, "invalid vector string") }
}
impl std::error::Error for ParseVectorError {
    fn description(&self) -> &str { "invalid vector string" }
    fn cause(&self) -> Option<&std::error::Error> { None }
}

fn parse_obj_vertex(mut tokens: std::str::SplitWhitespace)
        -> Result<nc::math::Point<f64>, Box<std::error::Error>> {
    let mut v = unsafe { nc::math::Point::<f64>::new_uninitialized() };
    for i in 0..3 {
        let token = tokens.next().ok_or(ParseVectorError)?;
        v[i] = token.parse::<f64>()?;
    }
    Ok(v)
}

fn parse_obj_face(mut tokens: std::str::SplitWhitespace)
        -> Result<na::geometry::Point3<usize>, Box<std::error::Error>> {
    let mut f = unsafe { na::geometry::Point3::<usize>::new_uninitialized() };
    for i in 0..3 {
        let token = tokens.next().ok_or(ParseVectorError)?;
        f[i] = token.split('/').next().ok_or(ParseVectorError)?.parse::<usize>()?;
    }
    Ok(f)
}

fn parse_obj(filename: &str) -> (Vec<nc::math::Point<f64>>, Vec<na::geometry::Point3<usize>>) {
    use std::io::{BufReader, BufRead};

    let file = std::fs::File::open(filename).expect(&format!("unable to open {}", filename));

    let mut points = Vec::<nc::math::Point<f64>>::new();
    let mut indices = Vec::<na::geometry::Point3<usize>>::new();

    for line in BufReader::new(file).lines() {
        let line = line.expect(&format!("unable to read line in {}", filename));
        let mut tokens = line.split_whitespace();
        match tokens.next() {
            Some("v") => {
                let v = parse_obj_vertex(tokens).expect(&format!("unable to parse vertex from {}", line));
                points.push(v);
            },
            Some("f") => {
                let f = parse_obj_face(tokens).expect(&format!("unable to parse face from {}", line));
                indices.push(f);
            }
            _ => {}
        };
        println!("{}", line);
    }

    (points, indices)
}

#[no_mangle]
pub fn trimesh_file(filename: &str) -> *mut nc::shape::ShapeHandle<f64> {
    let (points, indices) = match &filename[filename.len()-4..] {
        ".obj" => { parse_obj(filename) },
        _ => { panic!("unsupported file format") }
    };

    let trimesh = nc::shape::TriMesh::new(points, indices, None);
    let handle = nc::shape::ShapeHandle::new(trimesh);
    Box::into_raw(Box::new(handle))
}
