include_directories(
    ${OSG_INCLUDE_DIR}
    ${OSGGEO_INCLUDE_DIR}
)

SET ( RUNTIMELIBS
        ${OSG_GEO_OSG_LIBRARY}
        ${OSG_GEO_OPENTHREADS_LIBRARY}
        ${OSG_GEO_OSGUTIL_LIBRARY}
        ${OSG_GEO_OSGVIEWER_LIBRARY}
	${OSG_GEO_OSGGA_LIBRARY}
        ${OSG_GEO_OSGDB_LIBRARY} )

guess_runtime_library_dirs( RUNTIMELIBDIR ${RUNTIMELIBS} )

macro(add_example name)
    add_executable(${name} ${ARGN})
    create_target_launcher( ${name}
    	    RUNTIME_LIBRARY_DIRS
	    ${RUNTIMELIBDIR} )
    target_link_libraries(${name}
	${RUNTIMELIBS}
        osgGeo
    )
endmacro()

add_example(horizons horizons.cpp)
add_example(PolyLines PolyLines.cpp)
add_example(textureplane textureplane.cpp)
