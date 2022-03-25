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

namespace ncollide2d {

PYBIND11_MODULE(ncollide2d, m) {
  m.doc() = R"pbdoc(
    ncollide Python API
    ===================

    Python wrapper for the ncollide library.
  )pbdoc";

  py::module m_bounding_volume = m.def_submodule("bounding_volume");
  py::class_<ncollide2d::bounding_volume::AABB>(m_bounding_volume, "AABB")
      .def_property_readonly("maxs", &ncollide2d::bounding_volume::AABB::maxs)
      .def_property_readonly("mins", &ncollide2d::bounding_volume::AABB::mins);

  py::module m_shape = m.def_submodule("shape");
  py::class_<shape::Shape>(m_shape, "Shape");

  py::class_<shape::Ball, shape::Shape>(m_shape, "Ball")
      .def(py::init<double>(), "radius"_a)
      .def_property_readonly("radius", &shape::Ball::radius);

  py::add_ostream_redirect(m);
}

}  // namespace ncollide2d
