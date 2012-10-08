include_directories(
    ${OSG_INCLUDE_DIR}
    ${OSGGEO_INCLUDE_DIR}
)

SET ( RUNTIMELIBS
        ${OSGGEO_OSG_LIBRARY}
        ${OSGGEO_OPENTHREADS_LIBRARY}
        ${OSGGEO_OSGUTIL_LIBRARY}
        ${OSGGEO_OSGVIEWER_LIBRARY}
	${OSGGEO_OSGGA_LIBRARY}
        ${OSGGEO_OSGDB_LIBRARY} )

guess_runtime_library_dirs( RUNTIMELIBDIR ${RUNTIMELIBS} )

macro(add_example name)
    add_executable(${name} ${ARGN})
    IF( OSGGEO_MAKE_LAUNCHERS )
	create_target_launcher( ${name}
		RUNTIME_LIBRARY_DIRS
		${RUNTIMELIBDIR} )
    ENDIF()
    target_link_libraries(${name}
	${RUNTIMELIBS}
        osgGeo
    )
endmacro()

add_example(horizons horizons.cpp)
add_example(PolyLines PolyLines.cpp)
add_example(textureplane textureplane.cpp)
