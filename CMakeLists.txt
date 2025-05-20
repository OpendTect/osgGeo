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

cmake_minimum_required ( VERSION 3.27.0 )

cmake_policy( SET CMP0048 NEW )
cmake_policy( SET CMP0091 NEW )
#Dummy version, will be retrieved from find_package(OpenSceneGraph)
project( "osgGeo" VERSION 0.0.0 )

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED True )

cmake_policy( SET CMP0011 NEW )
cmake_policy( SET CMP0017 NEW )
cmake_policy( SET CMP0057 NEW )
cmake_policy( SET CMP0074 NEW )
cmake_policy( SET CMP0144 NEW )

set( ENV{OSG_ROOT} "${OSG_ROOT}" )
set( OSG_FIND_MODULES osgDB;osgGA;osgUtil;osgManipulator;osgWidget;osgViewer;osgVolume;osgText;osgSim )
find_package( OpenSceneGraph REQUIRED COMPONENTS ${OSG_FIND_MODULES} GLOBAL )

if ( UNIX AND NOT APPLE AND NOT DEFINED CMAKE_INSTALL_RPATH )
    set( CMAKE_INSTALL_RPATH "\\$ORIGIN" )
endif()

project( "osgGeo"
	VERSION ${OPENSCENEGRAPH_VERSION}
	DESCRIPTION "osgGeo" )

set ( BUILD_EXAMPLES "" CACHE BOOL "Build examples" )

list( APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules")
enable_testing()
include( CTest )

include( CreateLaunchers )
include( osgGeoCommon )
include( osgGeoSetupOSG )

include_directories( ${osgGeo_SOURCE_DIR}/src )
include_directories( SYSTEM ${OSG_INCLUDE_DIR} ${QT_HEADERS_DIR} )

if ( APPLE )
    install( FILES
	    CMakeModules/FindosgGeo.cmake
	    CMakeModules/FindOSG.cmake
	    DESTINATION Contents/Resources/share/CMakeModules )
else()
    install( FILES
	    CMakeModules/FindosgGeo.cmake
	    CMakeModules/FindOSG.cmake
	    DESTINATION share/CMakeModules )
endif()

add_subdirectory(src)

if( BUILD_EXAMPLES )
    add_subdirectory(examples)
endif( BUILD_EXAMPLES )
