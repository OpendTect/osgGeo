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

set( LIB_NAME osgGeo )
include( "../../CMakeModules/ListFilter.cmake" )

if ( (NOT DEFINED OPENSCENEGRAPH_FOUND) )
    find_package( OpenSceneGraph REQUIRED osgDB osgDA osgUtil osgManipulator osgWidget osgViewer osgVolume osgText osgSim )
endif()

if ( UNIX )
  set( OpenGL_GL_PREFERENCE "LEGACY" )
endif()
find_package( OpenGL REQUIRED )

if ( WIN32 )
    add_definitions( "/EHsc" "-DWIN32" )
endif()

if ( UNIX )
    include (CheckCXXSourceCompiles)

    check_cxx_source_compiles(
	"${OPENGL_HEADER1}
	${OPENGL_HEADER2}
	int main() { GLint64 test = 2; return (int) test; }"
	GL_HEADER_HAS_GLINT64
    )

    check_cxx_source_compiles(
	"${OPENGL_HEADER1}
	${OPENGL_HEADER2}
	int main() { GLuint64 test = 2; return (int) test; }"
	GL_HEADER_HAS_GLUINT64
    )

    if ( GL_HEADER_HAS_GLINT64  EQUAL 1 )
	add_definitions( "-DGL_HEADER_HAS_GLINT64" )
    endif() 

    if ( OD_GL_HEADER_HAS_GLUINT64 EQUAL 1 )
	add_definitions( "-DGL_HEADER_HAS_GLUINT64" )
    endif()
endif()

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/..
		     ${CMAKE_CURRENT_BINARY_DIR}/.. )
include_directories( SYSTEM ${OSG_INCLUDE_DIR} )

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/Config )


set ( LIB_PUBLIC_HEADERS 
    AxesNode
    Export
    Callback
    Common
    ComputeBoundsVisitor
    Config
    Draggers
    GLInfo
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
    Text
    TexturePlane
    TexturePanelStrip
    ThreadGroup
    ThumbWheel
    TiledOffScreenRenderer
    TrackballManipulator
    TubeWellLog
    Vec2i
    VolumeTechniques
    WellLog) 

add_library( ${LIB_NAME} SHARED
    ${LIB_PUBLIC_HEADERS}
    AxesNode.cpp
    Callback.cpp
    Draggers.cpp
    GLInfo.cpp
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
    Text.cpp
    TexturePlane.cpp
    TexturePanelStrip.cpp
    ThumbWheel.cpp
    TiledOffScreenRenderer.cpp
    TrackballManipulator.cpp 
    TubeWellLog.cpp
    VolumeTechniques.cpp
    WellLog.cpp)
set( OSG_GEODEPS ${OPENSCENEGRAPH_LIBRARIES} )
OD_FILTER_LIBRARIES( OSG_GEODEPS ${CMAKE_BUILD_TYPE} )
target_link_libraries(
    ${LIB_NAME}
    ${OSG_GEODEPS}
    ${OPENGL_gl_LIBRARY}
)

if ( OSGGEO_INSTDIR_LIB_DEBUG )
    if ( WIN32 )
	install( TARGETS ${LIB_NAME} 
	    CONFIGURATIONS "Debug"
	    RUNTIME DESTINATION ${OSGGEO_INSTDIR_LIB_DEBUG} )
	install( TARGETS ${LIB_NAME} 
	    CONFIGURATIONS "Release"
	    RUNTIME DESTINATION ${OSGGEO_INSTDIR_LIB_RELEASE} )
    else()
	install( TARGETS ${LIB_NAME} 
	    CONFIGURATIONS "Debug"
	    LIBRARY DESTINATION ${OSGGEO_INSTDIR_LIB_DEBUG} )
	install( TARGETS ${LIB_NAME} 
	    CONFIGURATIONS "Release"
	    LIBRARY DESTINATION ${OSGGEO_INSTDIR_LIB_RELEASE} )
    endif()
else()
    if ( APPLE )
	install(FILES
	    ${LIB_PUBLIC_HEADERS}
	    DESTINATION Contents/Resources/include/${LIB_NAME} )
	install(TARGETS
	    ${LIB_NAME}
	    LIBRARY DESTINATION Contents/MacOS )
    else()
	install(FILES
	    ${LIB_PUBLIC_HEADERS}
	    DESTINATION include/${LIB_NAME} )
	if ( WIN32 )
	    install(TARGETS
		${LIB_NAME}
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin )
	else()
	    install(TARGETS
		${LIB_NAME}
		LIBRARY DESTINATION lib )
	endif()
    endif()
endif()
