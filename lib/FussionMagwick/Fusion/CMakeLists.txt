project(libFusion)
cmake_minimum_required(VERSION 1.16)

file(GLOB_RECURSE files "*.c")

add_library(Fusion ${files})
set( CMAKE_VERBOSE_MAKEFILE on )

if(UNIX AND NOT APPLE)
    target_link_libraries(Fusion m) # link math library for Linux
endif()
