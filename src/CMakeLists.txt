############################################################
# CMakeLists for the ncollide-cpp library
#
# Copyright 2019. All Rights Reserved.
#
# Created: April 15, 2019
# Authors: Toki Migimatsu
############################################################

# Build ncollide.
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CARGO_CMD cargo build)
    set(TARGET_DIR "debug")
else ()
    set(CARGO_CMD cargo build --release)
    set(TARGET_DIR "release")
endif ()

add_custom_target(ncollide ALL
    COMMENT "Compiling ncollide module"
    COMMAND CARGO_TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR} ${CARGO_CMD} 
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(ncollide PROPERTIES LOCATION ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR})

# Build ncollide_cpp.
set(LIB_NAME ${PROJECT_NAME})

# Create the library.
add_library(${LIB_NAME} SHARED)
add_library(${LIB_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

target_compile_features(${LIB_NAME} PUBLIC cxx_std_17)
set_target_properties(${LIB_NAME} PROPERTIES CXX_EXTENSIONS OFF)

# Set sources.
target_sources(${LIB_NAME}
  PRIVATE
    ncollide.cc
    ncollide2d.cc
    ncollide3d.cc
)

# Set include directories.
set(LIB_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
target_include_directories(${LIB_NAME}
  PUBLIC
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
    "$<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>"
  PRIVATE
    "${LIB_INCLUDE_DIR}/ncollide_cpp"
    "${PROJECT_SOURCE_DIR}/src"
)

# Link library dependencies.
lib_add_subdirectory(ctrl_utils)
ctrl_utils_add_subdirectory(Eigen3)
get_target_property(NCOLLIDE_DIR ncollide LOCATION)
find_package(Threads REQUIRED)
target_link_libraries(${LIB_NAME}
  PUBLIC
    ${NCOLLIDE_DIR}/libncollide2d_cpp.a
    ${NCOLLIDE_DIR}/libncollide3d_cpp.a
    ctrl_utils::ctrl_utils
    Eigen3::Eigen
    Threads::Threads
    ${CMAKE_DL_LIBS}
)
add_dependencies(${LIB_NAME} ncollide)

# Build python wrapper.
if(${LIB_CMAKE_NAME}_BUILD_PYTHON)
    add_subdirectory(python)
endif()

# Install the library.
install(TARGETS ${LIB_NAME}
    EXPORT ${LIB_NAME}Targets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

# Install the headers.
install(DIRECTORY "${LIB_INCLUDE_DIR}/${LIB_NAME}"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

# Create LibConfig.cmake.
set(LIB_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${LIB_NAME}")
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${LIB_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${LIB_NAME}Config.cmake"
    INSTALL_DESTINATION "${LIB_CONFIG_INSTALL_DIR}"
)

# Create LibConfigVersion.cmake.
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/${LIB_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install LibConfig.cmake and LibConfigVersion.cmake.
install(FILES
    "${PROJECT_BINARY_DIR}/${LIB_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${LIB_NAME}ConfigVersion.cmake"
    DESTINATION ${LIB_CONFIG_INSTALL_DIR}
)

# Export targets to LibTargets.cmake.
install(EXPORT ${LIB_NAME}Targets
    DESTINATION ${LIB_CONFIG_INSTALL_DIR}
    NAMESPACE ${LIB_NAME}::
)

# Export targets to LibTargets.cmake for the build tree.
export(TARGETS ${LIB_NAME}
    NAMESPACE ${LIB_NAME}::
    FILE ${LIB_NAME}Targets.cmake
)
