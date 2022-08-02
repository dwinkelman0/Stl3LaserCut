# Copyright 2022 by Daniel Winkelman. All rights reserved.

cmake_minimum_required (VERSION 3.16)

set (CMAKE_BUILD_TYPE Debug)
set (CMAKE_CXX_STANDARD 20)

project (Stl3LaserCut CXX)
set (Target_lib stl3lasercut)
set (Target_client stl3lasercut-client)
set (Target_tests stl3lasercut-tests)

include (FetchContent)
FetchContent_Declare (googletest URL https://github.com/google/googletest/archive/refs/tags/release-1.11.0.zip)
FetchContent_MakeAvailable (googletest)
enable_testing ()

FetchContent_Declare (algo
    GIT_REPOSITORY https://github.com/dwinkelman0/Algo
    GIT_TAG v0.4
    CMAKE_ARGS -D DISABLE_TESTS=1
)
FetchContent_MakeAvailable (algo)
FetchContent_GetProperties (algo)

include_directories (src)
include_directories (${googletest_SOURCE_DIR}/googletest/include)
include_directories (${algo_SOURCE_DIR}/src)

file (GLOB LIB_SOURCES src/stl3lasercut/*.cc)
add_library (${Target_lib} ${LIB_SOURCES})
target_link_libraries (${Target_lib} algo)

file (GLOB CLIENT_SOURCES client/*.cc)
add_executable (${Target_client} ${CLIENT_SOURCES})
target_link_libraries (${Target_client} ${Target_lib})

file (GLOB TESTS_SOURCES tests/*.cc)
add_executable (${Target_tests} ${TESTS_SOURCES})
target_link_libraries (${Target_tests} ${Target_lib} gtest_main)

include (GoogleTest)
gtest_discover_tests (${Target_tests})
