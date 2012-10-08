include_directories(
    ${OSG_INCLUDE_DIR}
    ${OSGGEO_INCLUDE_DIR}
)

macro(add_example name)
    add_executable(${name} ${ARGN})
    create_target_launcher( ${name}
    	    RUNTIME_LIBRARY_DIRS
    	    ${OSG_GEO_OSG_LIBRARY_PATH}/../bin )
    target_link_libraries(${name}
        ${OSG_GEO_OSG_LIBRARY}
        ${OSG_GEO_OPENTHREADS_LIBRARY}
        ${OSG_GEO_OSGUTIL_LIBRARY}
        ${OSG_GEO_OSGVIEWER_LIBRARY}
        ${OSG_GEO_OSGDB_LIBRARY}
        osgGeo
    )
endmacro()

add_example(horizons horizons.cpp)
add_example(PolyLines PolyLines.cpp)
add_example(textureplane textureplane.cpp)
