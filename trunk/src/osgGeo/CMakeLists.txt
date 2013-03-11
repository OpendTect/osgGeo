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

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/Config)

set ( LIB_PUBLIC_HEADERS 
    AutoTransform
    AxesNode
    Line3
    Export
    Common
    Config
    Horizon3DBase
    Horizon3D
    Horizon3D2
    LayeredTexture
    MarkerSet
    PlaneWellLog
    PolyLine
    OneSideRender
    RandomLine
    TexturePlane
    TubeWellLog
    TrackballManipulator
    Vec2i
    ThumbWheel
    WellLog )

add_library( ${LIB_NAME} SHARED
    ${LIB_PUBLIC_HEADERS}
    AutoTransform.cpp
    AxesNode.cpp
    Palette.cpp
    PlaneWellLog
    ShaderUtility.cpp
    PolyLine.cpp
    Horizon3DBase.cpp
    Horizon3D.cpp
    Horizon3D2.cpp
    LayeredTexture.cpp
    MarkerSet.cpp
    PlaneWellLog.cpp
    OneSideRender.cpp
    Line3.cpp
    RandomLine.cpp
    TexturePlane.cpp
    TubeWellLog.cpp
    ThumbWheel.cpp
    TrackballManipulator.cpp 
    WellLog.cpp )
target_link_libraries(
    ${LIB_NAME}
    ${OSGGEO_OSG_LIBRARY}
    ${OSGGEO_OPENTHREADS_LIBRARY}
    ${OSGGEO_OSGUTIL_LIBRARY}
    ${OSGGEO_OSGGA_LIBRARY}
    ${OSGGEO_OSGMANIPULATOR_LIBRARY}
    ${OSGGEO_OSGDB_LIBRARY}
    ${OSGGEO_OSGTEXT_LIBRARY})

install(FILES
    ${LIB_PUBLIC_HEADERS}
    DESTINATION include/${LIB_NAME} )

install(TARGETS
    ${LIB_NAME}
    RUNTIME DESTINATION bin${OSGGEO_LIB_POSTFIX}
    LIBRARY DESTINATION lib${OSGGEO_LIB_POSTFIX}
    ARCHIVE DESTINATION lib${OSGGEO_LIB_POSTFIX} )
