############################################################
# CMakeLists for the ncollide-cpp library
#
# Copyright 2019. All Rights Reserved.
#
# Created: April 15, 2019
# Authors: Toki Migimatsu
############################################################

add_test(NAME ncollide_test
    COMMAND cargo test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(ncollide_test main.cc)
target_link_libraries(ncollide_test
  PRIVATE
    ncollide_cpp::ncollide_cpp
)
