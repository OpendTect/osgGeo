cmake_minimum_required (VERSION 2.6)
PROJECT(osgGeo)

set (osgGeo_VERSION_MAJOR 0)
set (osgGeo_VERSION_MINOR 1)

SET(OSG_DIR "/Users/Shared/work/osg/build/OpenSceneGraph" CACHE LOCATION "where was osg built?")

FIND_PACKAGE(OSG)

INCLUDE_DIRECTORIES(
    ${osgGeo_SOURCE_DIR}/src
    ${OSG_DIR}/include
    ${OSG_INCLUDE_DIR}
)

add_subdirectory(src)

