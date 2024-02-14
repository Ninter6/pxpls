cmake_minimum_required(VERSION 3.20)

project(pxpls)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

find_package(raylib REQUIRED)

# library
add_subdirectory(pxpls)

# demo 2D
add_subdirectory(demo2D)
