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
# $Id$
#

set( LIB_NAME osgGeo )

if ( WIN32 )
    add_definitions( "/EHsc" "-DWIN32" )
endif()

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/..
		     ${CMAKE_CURRENT_BINARY_DIR}/.. )
include_directories( SYSTEM ${OSG_DIR}/include )

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/Config )


set ( LIB_PUBLIC_HEADERS 
    AutoTransform
    AxesNode
    Export
    Callback
    Common
    ComputeBoundsVisitor
    Config
    Draggers
    LayeredTexture
    LayerProcess
    Line3
    MarkerSet
    MarkerShape
    OneSideRender
    PlaneWellLog
    PolygonSelection
    PolyLine
    ScalarBar
    TabBoxDragger
    TabPlaneDragger
    TexturePlane
    TexturePanelStrip
    ThumbWheel
    TiledOffScreenRenderer
    TrackballManipulator
    TubeWellLog
    Vec2i
    VolumeTechniques
    WellLog) 

add_library( ${LIB_NAME} SHARED
    ${LIB_PUBLIC_HEADERS}
    AutoTransform.cpp
    AxesNode.cpp
    Callback.cpp
    Draggers.cpp
    Palette.cpp
    PlaneWellLog
    ShaderUtility.cpp
    PolygonSelection.cpp
    PolyLine.cpp
    LayeredTexture.cpp
    LayerProcess.cpp
    Line3.cpp
    MarkerSet.cpp
    MarkerShape.cpp
    OneSideRender.cpp
    PlaneWellLog.cpp
    ScalarBar.cpp
    TabPlaneDragger.cpp
    TabBoxDragger.cpp
    TexturePlane.cpp
    TexturePanelStrip.cpp
    ThumbWheel.cpp
    TiledOffScreenRenderer.cpp
    TrackballManipulator.cpp 
    TubeWellLog.cpp
    VolumeTechniques.cpp
    WellLog.cpp) 
target_link_libraries(
    ${LIB_NAME}
    ${OSG_LIBRARY}
    ${OSGVOLUME_LIBRARY}
    ${OPENTHREADS_LIBRARY}
    ${OSGUTIL_LIBRARY}
    ${OSGGA_LIBRARY}
    ${OSGMANIPULATOR_LIBRARY}
    ${OSGDB_LIBRARY}
    ${OSGTEXT_LIBRARY}
    ${OSGSIM_LIBRARY}
    ${OSGVIEWER_LIBRARY})

install(FILES
    ${LIB_PUBLIC_HEADERS}
    DESTINATION include/${LIB_NAME} )

install(TARGETS
    ${LIB_NAME}
    RUNTIME DESTINATION bin${OSGGEO_LIB_POSTFIX}
    LIBRARY DESTINATION lib${OSGGEO_LIB_POSTFIX}
    ARCHIVE DESTINATION lib${OSGGEO_LIB_POSTFIX} )
