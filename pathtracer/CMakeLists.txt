################################################################################
# Rev project
################################################################################
cmake_minimum_required (VERSION 2.6)
project(pathtracer)

include(../cmake/common.cmake)

# Collect all sources
file(GLOB_RECURSE PATHTRACER_SOURCE_FILES "*.cpp" "*.h" "*.inl" "../include/*.h")
GroupSources(src pathtracer)
add_executable(pathtracer ${PATHTRACER_SOURCE_FILES})
target_include_directories (pathtracer PUBLIC "../include")
target_include_directories (pathtracer PUBLIC "./")