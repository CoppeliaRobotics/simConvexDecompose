cmake_minimum_required(VERSION 3.15)
project(simExtConvexDecompose)

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
find_package(CoppeliaSim 4.5.0.0 REQUIRED)

coppeliasim_add_plugin(
    simExtConvexDecompose
    LEGACY
    SOURCES
    simExtConvexDecompose.cpp
    hacd/hacdGraph.cpp
    hacd/hacdHACD.cpp
    hacd/hacdICHull.cpp
    hacd/hacdManifoldMesh.cpp
    hacd/hacdMeshDecimator.cpp
    hacd/hacdMicroAllocator.cpp
    hacd/hacdRaycastMesh.cpp
    vhacd/src/btAlignedAllocator.cpp
    vhacd/src/btConvexHullComputer.cpp
    vhacd/src/FloatMath.cpp
    vhacd/src/VHACD.cpp
    vhacd/src/VHACD-ASYNC.cpp
    vhacd/src/vhacdICHull.cpp
    vhacd/src/vhacdManifoldMesh.cpp
    vhacd/src/vhacdMesh.cpp
    vhacd/src/vhacdRaycastMesh.cpp
    vhacd/src/vhacdVolume.cpp
)
target_compile_definitions(simExtConvexDecompose PRIVATE SIM_MATH_DOUBLE)
target_include_directories(simExtConvexDecompose PRIVATE hacd)
target_include_directories(simExtConvexDecompose PRIVATE vhacd/inc)
target_include_directories(simExtConvexDecompose PRIVATE vhacd/public)
target_include_directories(simExtConvexDecompose PRIVATE vhacd/src)
