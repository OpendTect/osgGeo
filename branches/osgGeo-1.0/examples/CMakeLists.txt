include_directories(
    ${OSG_INCLUDE_DIR}
    ${OSGGEO_INCLUDE_DIR}
)

set ( RUNTIMELIBS
        ${OSGGEO_OSG_LIBRARY}
	${OSGGEO_OSGVOLUME_LIBRARY}
        ${OSGGEO_OPENTHREADS_LIBRARY}
        ${OSGGEO_OSGUTIL_LIBRARY}
        ${OSGGEO_OSGVIEWER_LIBRARY}
	${OSGGEO_OSGGA_LIBRARY}
        ${OSGGEO_OSGDB_LIBRARY}
)

if ( USEQT )
    add_definitions( -DUSEQT )
    include_directories( ${QT_QTCORE_INCLUDE_DIR} )
    list( APPEND RUNTIMELIBS
	${QT_QTCORE_LIBRARY} 
	${QT_QTGUI_LIBRARY} 
        ${OSGGEO_OSGQT_LIBRARY} )
endif()

guess_runtime_library_dirs( RUNTIMELIBDIR ${RUNTIMELIBS} )

macro(add_example NAME)
    set ( EXEC_NAME example_${NAME} )
    add_executable( ${EXEC_NAME} ${ARGN})
    if ( OSGGEO_MAKE_LAUNCHERS )
	create_target_launcher( ${EXEC_NAME}
		RUNTIME_LIBRARY_DIRS
		${RUNTIMELIBDIR} )
    endif()
    target_link_libraries(${EXEC_NAME}
	${RUNTIMELIBS}
        osgGeo
    )
endmacro()

add_example( thumbwheel thumbwheel.cpp )
add_example( PolyLines PolyLines.cpp )
add_example( textureplane textureplane.cpp )
add_example( texturepanelstrip texturepanelstrip.cpp )
add_example( PolygonSelection PolygonSel.cpp )
add_example( AxesNode axesnode.cpp )

