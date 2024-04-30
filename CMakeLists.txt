cmake_minimum_required(VERSION 3.20)

project(pxpls)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

find_package(raylib REQUIRED)

# library
add_subdirectory(pxpls)

option(PXPLS_BUILD_DEMO "build demo" off)

# demo 2D
if (PXPLS_BUILD_DEMO)
    add_subdirectory(demo)
endif()