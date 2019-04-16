/**
 * rounded_cuboid.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;
extern crate ncollide2d as nc2;

use crate::nc;

/// SupportMap description of a rounded cuboid shape.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(PartialEq, Debug, Clone)]
pub struct RoundedCuboid<N: na::Real> {
    half_extents: nc::math::Vector<N>,
    radius: N,
}

impl<N: na::Real> RoundedCuboid<N> {
    /// Creates a new rounded cuboid.
    ///
    /// # Arguments:
    /// * `half_extents` - the half extents of the rounded cuboid.
    /// * `radius` - radius of the rounded part of the rounded cuboid.
    pub fn new(half_extents: nc::math::Vector<N>, radius: N) -> RoundedCuboid<N> {
        assert!(radius.is_positive());
        for i in 0..nc::math::DIM {
            assert!(half_extents[i].is_positive());
        }

        RoundedCuboid {
            half_extents: half_extents,
            radius: radius,
        }
    }

    /// The rounded cuboid half extents (half-widths along each axis).
    #[inline]
    pub fn half_extents(&self) -> &nc::math::Vector<N> {
        &self.half_extents
    }

    /// The radius of the rounded cuboid's rounded part.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius
    }

    /// The cuboid that, once dilated by `self.radius` yields this rounded cuboid.
    #[inline]
    pub fn cuboid(&self) -> nc::shape::Cuboid<N> {
        nc::shape::Cuboid::new(*self.half_extents())
    }

    // /// The contact preprocessor to be used for contact determination with this rounded cuboid.
    // #[inline]
    // pub fn contact_preprocessor(&self) -> impl ContactPreprocessor<N> {
    //     RoundedCuboidContactPreprocessor {
    //         radius: self.radius
    //     }
    // }
}

impl<N: na::Real> nc::shape::SupportMap<N> for RoundedCuboid<N> {
    #[inline]
    fn support_point(&self, m: &nc::math::Isometry<N>, dir: &nc::math::Vector<N>) -> nc::math::Point<N> {
        self.support_point_toward(m, &na::Unit::new_normalize(*dir))
    }

    #[inline]
    fn support_point_toward(&self, m: &nc::math::Isometry<N>, dir: &na::Unit<nc::math::Vector<N>>) -> nc::math::Point<N> {
        use nc::utils::IsometryOps;

        let local_dir = m.inverse_transform_vector(dir);

        let mut res = *self.half_extents();

        for i in 0..nc::math::DIM {
            if local_dir[i].is_negative() {
                res[i] = -res[i];
            }
        }

        m * nc::math::Point::from(res + local_dir * self.radius())
    }
}

impl<N: na::Real> nc::bounding_volume::HasBoundingVolume<N, nc::bounding_volume::AABB<N>>
for RoundedCuboid<N> {
    #[inline]
    fn bounding_volume(&self, m: &nc::math::Isometry<N>) -> nc::bounding_volume::AABB<N> {
        // let center = Point::from(m.translation().to_vector());
        // let ws_half_extents = m.absolute_transform_vector(self.half_extents() + self.radius());

        // AABB::from_half_extents(center, ws_half_extents)
        nc::bounding_volume::support_map_aabb(m, self)
    }
}

impl<N: na::Real> nc::bounding_volume::HasBoundingVolume<N, nc::bounding_volume::BoundingSphere<N>>
for RoundedCuboid<N> {
    #[inline]
    fn bounding_volume(&self, m: &nc::math::Isometry<N>)
            -> nc::bounding_volume::BoundingSphere<N> {
        let center = nc::math::Point::from(m.translation.vector);
        let radius = self.radius() + self.half_extents().norm();

        nc::bounding_volume::BoundingSphere::new(center, radius)
    }
}

impl<N: na::Real> nc::query::PointQuery<N> for RoundedCuboid<N> {
    #[inline]
    fn project_point(&self, m: &nc::math::Isometry<N>, pt: &nc::math::Point<N>, solid: bool)
            -> nc::query::PointProjection<N> {
        let cuboid = nc::shape::Cuboid::new(*self.half_extents());
        let proj = cuboid.project_point(m, pt, solid);
        let dproj = *pt - proj.point;

        if let Some((dir, dist)) = na::Unit::try_new_and_get(dproj, N::default_epsilon()) {
            let inside = dist <= self.radius();
            if solid && inside {
                nc::query::PointProjection::new(true, *pt)
            } else {
                nc::query::PointProjection::new(inside, proj.point + dir.into_inner() * self.radius())
            }
        } else {
            if solid {
                nc::query::PointProjection::new(true, *pt)
            } else {
                let mut dir: nc::math::Vector<N> = na::zero();
                dir[1] = na::one();
                dir = m * dir;
                nc::query::PointProjection::new(true, proj.point + dir * self.radius())
            }
        }
    }

    #[inline]
    fn project_point_with_feature(&self,
                                  m: &nc::math::Isometry<N>,
                                  pt: &nc::math::Point<N>)
            -> (nc::query::PointProjection<N>, nc::shape::FeatureId) {
        (self.project_point(m, pt, false), nc::shape::FeatureId::Face(0))
    }
}

impl<N: na::Real> nc::query::RayCast<N> for RoundedCuboid<N> {
    fn toi_and_normal_with_ray(&self,
                               m: &nc::math::Isometry<N>,
                               ray: &nc::query::Ray<N>,
                               solid: bool) -> Option<nc::query::RayIntersection<N>> {
        // use nc::query::ray_internal::implicit_toi_and_normal_with_ray;

        let ls_ray = ray.inverse_transform_by(m);
        let dl = nc::math::Point::from(self.half_extents().map(|x| -x - self.radius()));
        let dr = nc::math::Point::from(self.half_extents().map(|x| x + self.radius()));
        let res = nc::bounding_volume::AABB::new(dl, dr).toi_and_normal_with_ray(&nc::math::Isometry::identity(),
                                                                                 &ls_ray, solid);
        if res.is_none() {
            return res
        }
        let mut res = res.unwrap();

        // let res = implicit_toi_and_normal_with_ray(&nc::math::Isometry::identity(),
        //                                            self,
        //                                            &mut nc::query::algorithms::VoronoiSimplex::new(),
        //                                            &ls_ray,
        //                                            solid);

        let point = ls_ray.origin + ls_ray.dir * res.toi;

        // Compute direction axes in descending order of magnitude
        let mut dir = (0 as usize..nc::math::DIM).collect::<Vec<usize>>();
        dir.sort_by(|i, j| (-(point[*i].abs())).partial_cmp(&-(point[*j].abs())).unwrap());
        // for i in &mut dir {
        //     if point[*i] < na::zero() {
        //         *i += nc::math::DIM;
        //     }
        // }

        if point[dir[1]].abs() < self.half_extents()[dir[1]] &&
           point[dir[2]].abs() < self.half_extents()[dir[2]] {
            // Inside square face
            res.normal.fill(na::zero());
            res.normal[dir[0]] = na::one();
        } else if point[dir[2]].abs() > self.half_extents()[dir[2]] {
            // Inside corner
            let mut origin_sphere = self.half_extents().clone();
            for i in 0..3 {
                if point[dir[i]] < na::zero() {
                    origin_sphere[dir[i]] = -origin_sphere[dir[i]];
                }
            }
            let sphere = nc::shape::Ball::new(self.radius());
            let m = na::Isometry::from_parts(na::Translation::from(origin_sphere), na::UnitQuaternion::identity());

            match sphere.toi_and_normal_with_ray(&m, ray, solid) {
                Some(intersect) => { res = intersect; }
                None => return None
            };
            // TODO: Check corners if none
        } else {
            use nc2::query::RayCast;

            // Inside edge
            let mut origin_circle = nc2::math::Vector::new(self.half_extents()[dir[0]], self.half_extents()[dir[1]]);
            for i in 0..2 {
                if point[dir[i]] < na::zero() {
                    origin_circle[dir[i]] = -origin_circle[dir[i]];
                }
            }

            // toi with cylinder aligned with dir[2]
            let circle = nc2::shape::Ball::new(self.radius());
            let m = na::Isometry::from_parts(na::Translation::from(origin_circle), na::UnitComplex::identity());
            let ray2_origin = nc2::math::Point::new(ray.origin[dir[0]], ray.origin[dir[1]]);
            let ray2_dir = nc2::math::Vector::new(ray.dir[dir[0]], ray.dir[dir[1]]);
            let ray2 = nc2::query::Ray::new(ray2_origin, ray2_dir);
            let intersect = circle.toi_and_normal_with_ray(&m, &ray2, solid);
            if intersect.is_none() {
                return None
            }
            let intersect = intersect.unwrap();
            let point_cylinder = ray.origin + ray.dir * intersect.toi;
            if point_cylinder[dir[2]].abs() > self.half_extents()[dir[2]] {
                return None
            }

            res.normal[dir[0]] = intersect.normal[0];
            res.normal[dir[1]] = intersect.normal[1];
            res.normal[dir[2]] = na::zero();
            res.toi = intersect.toi;
        }
        res.normal = m * res.normal;
        Some(res)
    }
}

impl<N: na::Real> nc::shape::Shape<N> for RoundedCuboid<N> {

    #[inline]
    fn aabb(&self, m: &nc::math::Isometry<N>) -> nc::bounding_volume::AABB<N> {
        nc::bounding_volume::aabb(self, m)
    }

    #[inline]
    fn bounding_sphere(&self, m: &nc::math::Isometry<N>) -> nc::bounding_volume::BoundingSphere<N> {
        nc::bounding_volume::bounding_sphere(self, m)
    }

    #[inline]
    fn as_ray_cast(&self) -> Option<&nc::query::RayCast<N>> {
        Some(self)
    }

    #[inline]
    fn as_point_query(&self) -> Option<&nc::query::PointQuery<N>> {
        Some(self)
    }

    #[inline]
    fn as_support_map(&self) -> Option<&nc::shape::SupportMap<N>> {
        Some(self)
    }

    #[inline]
    fn is_support_map(&self) -> bool {
        true
    }

    // FIXME: this is wrong in theory but keep it this
    // way for now because of the way the ContactKinematic
    // currently works.
    fn tangent_cone_contains_dir(&self,
                                 _: nc::shape::FeatureId,
                                 _: &nc::math::Isometry<N>,
                                 _: Option<&[N]>,
                                 _: &na::Unit<nc::math::Vector<N>>) -> bool {
        false
    }
}

#[cfg(feature = "dim2")]
pub fn rounded_cuboid_new(x: f64, y: f64, radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let cuboid = RoundedCuboid::new(nc::math::Vector::new(x, y), radius);
    let handle = nc::shape::ShapeHandle::new(cuboid);
    Box::into_raw(Box::new(handle))
}

#[cfg(feature = "dim3")]
pub fn rounded_cuboid_new(x: f64, y: f64, z: f64, radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let cuboid = RoundedCuboid::new(nc::math::Vector::new(x, y, z), radius);
    let handle = nc::shape::ShapeHandle::new(cuboid);
    Box::into_raw(Box::new(handle))
}

pub fn rounded_cuboid_half_extents(shape: Option<&nc::shape::ShapeHandle<f64>>) -> *const f64 {
    use na::storage::Storage;

    let maybe_cuboid = shape.unwrap().as_shape::<RoundedCuboid<f64>>();
    match maybe_cuboid {
        Some(ref cuboid) => { cuboid.half_extents().data.ptr() },
        None => { std::ptr::null() }
    }
}
