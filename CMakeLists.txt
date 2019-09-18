# osgGeo - A collection of geoscientific extensions to OpenSceneGraph.
# Copyright 2011 dGB Beheer B.V.
# 
# osgGeo is free software; you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>
#
# All configurations should be set in osgGeoConfig.cmake
#
# No configuration settings should be modified below.

project( osgGeo )

set (osgGeo_VERSION_MAJOR 0)
set (osgGeo_VERSION_MINOR 1)

set ( BUILD_EXAMPLES "" CACHE BOOL "Build examples" )

cmake_minimum_required (VERSION 2.6)


list( APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules")
enable_testing()
include( CTest )

include( CreateLaunchers )
include( osgGeoCommon )
include( osgGeoSetupOSG )

if( BUILD_EXAMPLES )
    if ( APPLE )
	find_package( Qt4 REQUIRED )
	set( USEQT 1 )
    endif()
endif( BUILD_EXAMPLES )

include_directories( ${osgGeo_SOURCE_DIR}/src )
include_directories( SYSTEM ${OSG_INCLUDE_DIR} ${QT_HEADERS_DIR} )

install( FILES
	CMakeModules/FindosgGeo.cmake
	CMakeModules/FindOSG.cmake
	DESTINATION share/CMakeModules )

add_subdirectory(src)

if( BUILD_EXAMPLES )
    add_subdirectory(examples)
endif( BUILD_EXAMPLES )

#To avoid warnings in continious integration.
set ( DUMMY ${COINDIR} ${CTEST_MODEL} ${OSGGEO_DIR} ${QTDIR} ${CMAKE_BUILD_TYPE} )

