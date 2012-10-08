# Locate gdal
# This module defines
# OSGGEO_LIBRARY
# OSGGEO_FOUND, if false, do not try to link to gdal 
# OSGGEO_INCLUDE_DIR, where to find the headers
#
# $OSGGEO_DIR is an environment variable that would
# correspond to the ./configure --prefix=$OSGGEO_DIR
#
# Created by Robert Osfield. 

FIND_PATH(OSGGEO_INCLUDE_DIR osgGeo/Horizon3D
    ${OSGGEO_DIR}/include
    $ENV{OSGGEO_DIR}/include
    NO_DEFAULT_PATH
)

FIND_PATH(OSGGEO_INCLUDE_DIR osgGeo/Horizon3D)

MACRO(FIND_OSGGEO_LIBRARY MYLIBRARY MYLIBRARYNAME)

    FIND_LIBRARY("${MYLIBRARY}_DEBUG"
        NAMES "${MYLIBRARYNAME}${CMAKE_DEBUG_POSTFIX}"
        PATHS
        ${OSGGEO_DIR}/lib/Debug
        ${OSGGEO_DIR}/lib64/Debug
        ${OSGGEO_DIR}/lib
        ${OSGGEO_DIR}/lib64
        $ENV{OSGGEO_DIR}/lib/debug
        $ENV{OSGGEO_DIR}/lib64/debug
        $ENV{OSGGEO_DIR}/lib
        $ENV{OSGGEO_DIR}/lib64
        $ENV{OSGGEO_DIR}
        $ENV{OSGGEODIR}/lib
        $ENV{OSGGEODIR}/lib64
        $ENV{OSGGEODIR}
        $ENV{OSGGEO_ROOT}/lib
        $ENV{OSGGEO_ROOT}/lib64
        NO_DEFAULT_PATH
    )

    FIND_LIBRARY("${MYLIBRARY}_DEBUG"
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
    
    FIND_LIBRARY(${MYLIBRARY}
        NAMES "${MYLIBRARYNAME}${CMAKE_RELEASE_POSTFIX}"
        PATHS
        ${OSGGEO_DIR}/lib/Release
        ${OSGGEO_DIR}/lib64/Release
        ${OSGGEO_DIR}/lib
        ${OSGGEO_DIR}/lib64
        $ENV{OSGGEO_DIR}/lib/Release
        $ENV{OSGGEO_DIR}/lib64/Release
        $ENV{OSGGEO_DIR}/lib
        $ENV{OSGGEO_DIR}/lib64
        $ENV{OSGGEO_DIR}
        $ENV{OSGGEODIR}/lib
        $ENV{OSGGEODIR}/lib64
        $ENV{OSGGEODIR}
        $ENV{OSGGEO_ROOT}/lib
        $ENV{OSGGEO_ROOT}/lib64
        NO_DEFAULT_PATH
    )

    FIND_LIBRARY(${MYLIBRARY}
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

    IF( NOT ${MYLIBRARY}_DEBUG)
        IF(MYLIBRARY)
            SET(${MYLIBRARY}_DEBUG ${MYLIBRARY})
        ENDIF(MYLIBRARY)
    ELSE()
	IF( NOT MYLIBRARY )
            SET(${MYLIBRARY} ${${MYLIBRARY}_DEBUG} )
        ENDIF(NOT MYLIBRARY)
    ENDIF( NOT ${MYLIBRARY}_DEBUG)
           
ENDMACRO(FIND_OSGGEO_LIBRARY LIBRARY LIBRARYNAME)

FIND_OSGGEO_LIBRARY(OSGGEO_LIBRARY osgGeo)

SET(OSGGEO_FOUND "NO")
IF(OSGGEO_LIBRARY AND OSGGEO_INCLUDE_DIR)
    SET(OSGGEO_FOUND "YES")
ENDIF(OSGGEO_LIBRARY AND OSGGEO_INCLUDE_DIR)
