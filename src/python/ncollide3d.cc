/**
 * ncollide.cc
 *
 * Copyright 2022. All Rights Reserved.
 *
 * Created: March 24, 2022
 * Authors: Toki Migimatsu
 */

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ncollide_cpp/ncollide.h"

namespace {

namespace py = pybind11;
using namespace pybind11::literals;  // NOLINT(google-build-using-namespace)

}  // namespace

namespace ncollide3d {

PYBIND11_MODULE(ncollide3d, m) {
  m.doc() = R"pbdoc(
    ncollide Python API
    ===================

    Python wrapper for the ncollide library.
  )pbdoc";

  // bounding_volume submodule
  py::module m_bounding_volume = m.def_submodule("bounding_volume");
  py::class_<bounding_volume::AABB>(m_bounding_volume, "AABB")
      .def_property_readonly("maxs", &bounding_volume::AABB::maxs)
      .def_property_readonly("mins", &bounding_volume::AABB::mins);
  m_bounding_volume.def("aabb", &bounding_volume::aabb, "g"_a,
                        "m"_a = Eigen::Isometry3d::Identity());

  // shape submodule
  py::module m_shape = m.def_submodule("shape");
  py::class_<shape::Shape>(m_shape, "Shape")
      .def("aabb", &shape::Shape::aabb, "m"_a = Eigen::Isometry3d::Identity());

  py::class_<shape::Ball, shape::Shape>(m_shape, "Ball")
      .def(py::init<double>(), "radius"_a)
      .def_property_readonly("radius", &shape::Ball::radius)
      .def("project_2d", &shape::Ball::project_2d);

  py::class_<shape::TriMesh, shape::Shape>(m_shape, "TriMesh")
      .def(py::init<const std::string&>(), "filename"_a)
      .def("project_2d", &shape::TriMesh::project_2d)
      .def_property_readonly("num_points", &shape::TriMesh::num_points)
      .def("point", &shape::TriMesh::point, "i"_a);

  // query submodule
  py::module m_query = m.def_submodule("query");
  py::class_<query::Contact>(m_query, "Contact")
      .def_readwrite("world1", &query::Contact::world1)
      .def_readwrite("world2", &query::Contact::world2)
      .def_readwrite("normal", &query::Contact::normal)
      .def_readwrite("depth", &query::Contact::depth);

  m_query.def("distance", &query::distance, "m1"_a, "g1"_a, "m2"_a, "g2"_a)
      .def("contact", &query::contact, "m1"_a, "g1"_a, "m2"_a, "g2"_a,
           "prediction"_a);

  py::add_ostream_redirect(m);
}

}  // namespace ncollide3d
