############################################################
# CMakeLists for the ncollide-cpp library
#
# Copyright 2019. All Rights Reserved.
#
# Created: April 15, 2019
# Authors: Toki Migimatsu
############################################################

cmake_minimum_required(VERSION 3.11)

# Define project
project(ncollide_cpp
    VERSION 0.2.0
    DESCRIPTION "C++ wrapper for ncollide"
    LANGUAGES CXX)
string(TOUPPER ${PROJECT_NAME} LIB_CMAKE_NAME)
set(LIB_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# Include custom CMake utilities.
include(cmake/utils.cmake)

# Detect whether this project is built by itself or included as a subdirectory.
lib_dependent_option(${LIB_CMAKE_NAME}_MAIN_BUILD
    "Build ${PROJECT_NAME} as a main project."
    "CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME"
)

# Generate a compilation database for autocompletion.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON) # TODO: Remove

# Use testing framework.
if(${LIB_CMAKE_NAME}_MAIN_BUILD)
    include(CTest)
endif()

# Define CMake options.
lib_option(BUILD_PYTHON "Build Python library." ON)
lib_option(BUILD_TESTING "Build tests." ON)

# Set default build type to release.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set output directories.
if(${LIB_CMAKE_NAME}_MAIN_BUILD)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
endif()

# Standardize install directories.
include(GNUInstallDirs)

# Build the library.
add_subdirectory(src)

# Build tests.
if(${LIB_CMAKE_NAME}_BUILD_TESTING)
    add_subdirectory(tests)
endif()
