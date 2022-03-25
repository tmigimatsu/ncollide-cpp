############################################################
# CMakeLists for Pybind11
#
# Copyright 2022. All Rights Reserved.
#
# Created: March 24, 2022
# Authors: Toki Migimatsu
############################################################

project(ncollide_py)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIB_SOURCE_DIR}/ncollide")

ctrl_utils_add_subdirectory(pybind11)

pybind11_add_module(ncollide2d ncollide2d.cc)
pybind11_add_module(ncollide3d ncollide3d.cc)

target_link_libraries(ncollide2d
  PUBLIC
    pybind11::pybind11
    ncollide_cpp::ncollide_cpp
)
target_link_libraries(ncollide3d
  PUBLIC
    pybind11::pybind11
    ncollide_cpp::ncollide_cpp
)

# Put binary for installation temporarily in build/src/python.
install(TARGETS ncollide2d
    LIBRARY DESTINATION "${PROJECT_BINARY_DIR}"
)
install(TARGETS ncollide3d
    LIBRARY DESTINATION "${PROJECT_BINARY_DIR}"
)
