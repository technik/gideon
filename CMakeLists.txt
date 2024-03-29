################################################################################
# Rev project
# Sample project that creates a simple scene graph
################################################################################
cmake_minimum_required (VERSION 3.10)
project(gideon)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_CXX_STANDARD 20)

# Visual studio specifics
if(MSVC)
	add_definitions(-DNOMINMAX)
endif(MSVC)

# Clasify sources according to folder structure. Useful for having nice visual studio filters.
# This macro is derived from http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir dirLabel)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child} ${dirLabel}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${dirLabel})
			source_group(${groupname} FILES
				${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

################################################################################
# Actual pathtracer code
################################################################################

# Collect all sources
file(GLOB_RECURSE PATHTRACER_SOURCE_FILES "pathtracer/*.cpp" "pathtracer/*.h" "pathtracer/*.inl" "include/*.h" "include/*.hpp")
GroupSources(pathtracer pathtracer)
GroupSources(include include)
add_executable(pathtracer ${PATHTRACER_SOURCE_FILES})
target_include_directories (pathtracer PUBLIC "include")
target_include_directories (pathtracer PUBLIC "pathtracer")

################################################################################
# tests code
################################################################################
enable_testing()

include_directories ("include")
include_directories ("pathtracer")

add_executable(algebraTest
	test/unit/algebraTest.cpp
	pathtracer/math/matrix.cpp)
set_target_properties(algebraTest PROPERTIES FOLDER test)
add_test(algebra_unit_test algebraTest)

add_executable(collisionTest
    test/unit/collisionTest.cpp
    pathtracer/collision/CWBVH.cpp)
set_target_properties(collisionTest PROPERTIES FOLDER test)
add_test(collision_unit_test collisionTest)


add_executable(tlasTest
    test/unit/tlasTest.cpp
	pathtracer/math/matrix.cpp
    pathtracer/collision/CWBVH.cpp
    pathtracer/collision/TLAS.cpp)
set_target_properties(tlasTest PROPERTIES FOLDER test)
add_test(tlas_unit_test tlasTest)