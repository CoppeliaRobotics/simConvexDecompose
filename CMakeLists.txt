cmake_minimum_required(VERSION 3.15)
project(simConvexDecompose)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED)

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.5.1.7 REQUIRED)

coppeliasim_add_plugin(
    simConvexDecompose
    LEGACY
    SOURCES
    sourceCode/simConvexDecompose.cpp
    external/hacd/hacdGraph.cpp
    external/hacd/hacdHACD.cpp
    external/hacd/hacdICHull.cpp
    external/hacd/hacdManifoldMesh.cpp
    external/hacd/hacdMeshDecimator.cpp
    external/hacd/hacdMicroAllocator.cpp
    external/hacd/hacdRaycastMesh.cpp
    external/vhacd/src/btAlignedAllocator.cpp
    external/vhacd/src/btConvexHullComputer.cpp
    external/vhacd/src/FloatMath.cpp
    external/vhacd/src/VHACD.cpp
    external/vhacd/src/VHACD-ASYNC.cpp
    external/vhacd/src/vhacdICHull.cpp
    external/vhacd/src/vhacdManifoldMesh.cpp
    external/vhacd/src/vhacdMesh.cpp
    external/vhacd/src/vhacdRaycastMesh.cpp
    external/vhacd/src/vhacdVolume.cpp
)
target_compile_definitions(simConvexDecompose PRIVATE SIM_MATH_DOUBLE)
target_include_directories(simConvexDecompose PRIVATE external/hacd)
target_include_directories(simConvexDecompose PRIVATE external/vhacd/inc)
target_include_directories(simConvexDecompose PRIVATE external/vhacd/public)
target_include_directories(simConvexDecompose PRIVATE external/vhacd/src)
coppeliasim_add_lua(lua/simConvexDecompose.lua)
