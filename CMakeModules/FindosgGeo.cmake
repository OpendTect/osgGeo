# Locate gdal
# This module defines
# OSGGEO_LIBRARY
# OSGGEO_FOUND, if false, do not try to link to gdal 
# OSGGEO_INCLUDE_DIR, where to find the headers
#
# $osgGeo_DIR is an environment variable that would
# correspond to the ./configure --prefix=$osgGeo_DIR
#
# Created by Robert Osfield. 

find_path(OSGGEO_INCLUDE_DIR osgGeo/Horizon3D
    ${osgGeo_DIR}/include
    ${osgGeo_DIR}/src
    $ENV{osgGeo_DIR}/include
    NO_DEFAULT_PATH
)

find_path(OSGGEO_INCLUDE_DIR osgGeo/Horizon3D)

macro(FIND_OSGGEO_LIBRARY MYLIBRARY MYLIBRARYNAME)

    find_library("${MYLIBRARY}_DEBUG"
        NAMES "${MYLIBRARYNAME}${CMAKE_DEBUG_POSTFIX}"
        PATHS
        ${osgGeo_DIR}/lib/Debug
        ${osgGeo_DIR}/lib64/Debug
        ${osgGeo_DIR}/lib
        ${osgGeo_DIR}/lib64
        ${osgGeo_DIR}/src/osgGeo/Debug
        ${osgGeo_DIR}/src/osgGeo
        $ENV{osgGeo_DIR}/lib/debug
        $ENV{osgGeo_DIR}/lib64/debug
        $ENV{osgGeo_DIR}/lib
        $ENV{osgGeo_DIR}/lib64
        $ENV{osgGeo_DIR}
        $ENV{OSGGEODIR}/lib
        $ENV{OSGGEODIR}/lib64
        $ENV{OSGGEODIR}
        $ENV{OSGGEO_ROOT}/lib
        $ENV{OSGGEO_ROOT}/lib64
        NO_DEFAULT_PATH
    )

    find_library("${MYLIBRARY}_DEBUG"
        NAMES "${MYLIBRARYNAME}${CMAKE_DEBUG_POSTFIX}"
        PATHS
        ~/Library/Frameworks
        /Library/Frameworks
        /usr/local/lib
        /usr/local/lib64
        /usr/lib
        /usr/lib64
        /sw/lib
        /opt/local/lib
        /opt/csw/lib
        /opt/lib
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSGGEO_ROOT]/lib
        /usr/freeware/lib64
    )

    if ( ${MYLIBRARY}_DEBUG STREQUAL "${MYLIBRARY}_DEBUG-NOTFOUND" )
	unset(  ${MYLIBRARY}_DEBUG )
    endif()
    
    find_library(${MYLIBRARY}
        NAMES "${MYLIBRARYNAME}${CMAKE_RELEASE_POSTFIX}"
        PATHS
        ${osgGeo_DIR}/lib/Release
        ${osgGeo_DIR}/lib64/Release
        ${osgGeo_DIR}/lib
        ${osgGeo_DIR}/lib64
        ${osgGeo_DIR}/src/osgGeo/Release
        ${osgGeo_DIR}/src/osgGeo
        $ENV{osgGeo_DIR}/lib/Release
        $ENV{osgGeo_DIR}/lib64/Release
        $ENV{osgGeo_DIR}/lib
        $ENV{osgGeo_DIR}/lib64
        $ENV{osgGeo_DIR}
        $ENV{OSGGEODIR}/lib
        $ENV{OSGGEODIR}/lib64
        $ENV{OSGGEODIR}
        $ENV{OSGGEO_ROOT}/lib
        $ENV{OSGGEO_ROOT}/lib64
        NO_DEFAULT_PATH
    )

    find_library(${MYLIBRARY}
        NAMES "${MYLIBRARYNAME}${CMAKE_RELEASE_POSTFIX}"
        PATHS
        ~/Library/Frameworks
        /Library/Frameworks
        /usr/local/lib
        /usr/local/lib64
        /usr/lib
        /usr/lib64
        /sw/lib
        /opt/local/lib
        /opt/csw/lib
        /opt/lib
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSGGEO_ROOT]/lib
        /usr/freeware/lib64
    )

    if ( ${MYLIBRARY} STREQUAL "${MYLIBRARY}-NOTFOUND" )
	unset( ${MYLIBRARY} )
    endif()

    if( NOT ${MYLIBRARY}_DEBUG )
        if( ${MYLIBRARY} )
            set(${MYLIBRARY}_DEBUG ${${MYLIBRARY}} )
        endif( ${MYLIBRARY} )
    endif( NOT ${MYLIBRARY}_DEBUG)

    if( NOT ${MYLIBRARY} )
	set(${MYLIBRARY} ${${MYLIBRARY}_DEBUG} )
    endif(NOT ${MYLIBRARY} )
           
endmacro(FIND_OSGGEO_LIBRARY MYLIBRARY MYLIBRARYNAME)

FIND_OSGGEO_LIBRARY(OSGGEO_LIBRARY osgGeo)

set(OSGGEO_FOUND "NO")
if ( OSGGEO_LIBRARY AND OSGGEO_INCLUDE_DIR)
    set(OSGGEO_FOUND "YES")
endif( OSGGEO_LIBRARY AND OSGGEO_INCLUDE_DIR)
