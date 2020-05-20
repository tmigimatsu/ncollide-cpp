############################################################
# CMakeLists for the ncollide-cpp library
#
# Copyright 2019. All Rights Reserved.
#
# Created: April 15, 2019
# Authors: Toki Migimatsu
############################################################

# Require 3.6 to support pkg_check_modules IMPORTED_TARGET
cmake_minimum_required(VERSION 3.6)

# Define project
project(ncollide_cpp VERSION 0.1.0 LANGUAGES CXX)

# Set CMake flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON) # TODO: Remove
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

# Define directories
set(NCOLLIDE_CPP_LIB ncollide_cpp)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(NCOLLIDE_CPP_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(NCOLLIDE_CPP_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(NCOLLIDE_CPP_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(NCOLLIDE_CPP_TEST_DIR ${PROJECT_SOURCE_DIR}/test)

# Create ncollide-cpp library
add_library(${NCOLLIDE_CPP_LIB} SHARED)

# Create namespaced alias for use inside the build tree
add_library(${NCOLLIDE_CPP_LIB}::${NCOLLIDE_CPP_LIB} ALIAS ${NCOLLIDE_CPP_LIB})

# Set compiler options
target_compile_options(${NCOLLIDE_CPP_LIB} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall>
    $<$<CONFIG:RELEASE>:-O3>
)

# Find dependencies
function(download_package PACKAGE_NAME)
    configure_file(${NCOLLIDE_CPP_CMAKE_DIR}/${PACKAGE_NAME}-CMakeLists.txt.in
        ${CMAKE_BINARY_DIR}/${PACKAGE_NAME}/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
endfunction()

find_package(ctrl_utils 1.2.0 QUIET)
if(NOT ctrl_utils_FOUND)
    download_package(ctrl_utils)
    find_package(ctrl_utils 1.2.0 REQUIRED)
endif()


if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CARGO_CMD cargo build)
    set(TARGET_DIR "debug")
else ()
    set(CARGO_CMD cargo build --release)
    set(TARGET_DIR "release")
endif ()

# set(NCOLLIDE_SO "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR}/libncollide_cpp.so")

# TODO: Use add_custom_command
add_custom_target(ncollide ALL
    COMMENT "Compiling ncollide module"
    COMMAND CARGO_TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR} ${CARGO_CMD} 
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(ncollide PROPERTIES LOCATION ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR})

add_test(NAME ncollide_test
    COMMAND cargo test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

get_target_property(NCOLLIDE_DIR ncollide LOCATION)
target_link_libraries(${NCOLLIDE_CPP_LIB}
    PUBLIC
        ${NCOLLIDE_DIR}/libncollide2d_cpp.a
        ${NCOLLIDE_DIR}/libncollide3d_cpp.a
        ctrl_utils::ctrl_utils
        dl
)
add_dependencies(${NCOLLIDE_CPP_LIB} ncollide)

# Set include directories
target_include_directories(${NCOLLIDE_CPP_LIB}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${NCOLLIDE_CPP_INCLUDE_DIR}>
    PRIVATE
        ${NCOLLIDE_CPP_INCLUDE_DIR}/ncollide_cpp
        ${NCOLLIDE_CPP_SRC_DIR})

# Attach sources
target_sources(${NCOLLIDE_CPP_LIB} PRIVATE
    ${NCOLLIDE_CPP_SRC_DIR}/ncollide.cc
    ${NCOLLIDE_CPP_SRC_DIR}/ncollide2d.cc
    ${NCOLLIDE_CPP_SRC_DIR}/ncollide3d.cc
)

add_executable(ncollide_test ${NCOLLIDE_CPP_TEST_DIR}/main.cc)
target_include_directories(ncollide_test
    PRIVATE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${NCOLLIDE_CPP_INCLUDE_DIR}>
)
target_link_libraries(ncollide_test
    PRIVATE
        ${NCOLLIDE_CPP_LIB}
)

# Use GNUInstallDirs to install libraries into correct locations on all platforms
include(GNUInstallDirs)

set(NCOLLIDE_CPP_TARGETS ${NCOLLIDE_CPP_LIB}Targets)
set(NCOLLIDE_CPP_CONFIG ${NCOLLIDE_CPP_LIB}Config)
set(NCOLLIDE_CPP_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${NCOLLIDE_CPP_LIB})

# Install the library
install(TARGETS ${NCOLLIDE_CPP_LIB}
    EXPORT ${NCOLLIDE_CPP_TARGETS}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install the headers
install(DIRECTORY ${NCOLLIDE_CPP_INCLUDE_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export targets to Targets.cmake
install(EXPORT ${NCOLLIDE_CPP_TARGETS}
    DESTINATION ${NCOLLIDE_CPP_CONFIG_INSTALL_DIR}
    NAMESPACE ${NCOLLIDE_CPP_LIB}::
    FILE ${NCOLLIDE_CPP_TARGETS}.cmake
)

# Create ConfigVersion.cmake
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_BINARY_DIR}/${NCOLLIDE_CPP_CONFIG}Version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Create Config.cmake
configure_package_config_file(${NCOLLIDE_CPP_CMAKE_DIR}/${NCOLLIDE_CPP_CONFIG}.cmake.in
    ${CMAKE_BINARY_DIR}/${NCOLLIDE_CPP_CONFIG}.cmake
    INSTALL_DESTINATION ${NCOLLIDE_CPP_CONFIG_INSTALL_DIR})

# Install config script files
install(FILES
    ${CMAKE_BINARY_DIR}/${NCOLLIDE_CPP_CONFIG}.cmake
    ${CMAKE_BINARY_DIR}/${NCOLLIDE_CPP_CONFIG}Version.cmake
    DESTINATION ${NCOLLIDE_CPP_CONFIG_INSTALL_DIR}
)

# Build tests
# include(CTest)
# if(BUILD_TESTING)
#     add_subdirectory(tests)
# endif()

export(TARGETS ${NCOLLIDE_CPP_LIB}
    NAMESPACE ${NCOLLIDE_CPP_LIB}::
    FILE ${CMAKE_BINARY_DIR}/${NCOLLIDE_CPP_TARGETS}.cmake)

# Register package in user registry
export(PACKAGE ${NCOLLIDE_CPP_LIB})

