SET(OSG_DIR "")


cmake_minimum_required (VERSION 2.6)

PROJECT(osgGeo)

set (osgGeo_VERSION_MAJOR 0)
set (osgGeo_VERSION_MINOR 1)


INCLUDE_DIRECTORIES(
    ${osgGeo_SOURCE_DIR}/src
    ${OSG_DIR}/include
)

add_subdirectory(src)

