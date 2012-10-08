include_directories(
    ${OSG_INCLUDE_DIR}
    ${OSGGEO_INCLUDE_DIR}
)

add_custom_target(osgGeoExamples)

macro(add_example name)
    add_executable(${name} EXCLUDE_FROM_ALL ${ARGN})
    target_link_libraries(${name}
        ${OSG_GEO_OSG_LIBRARY}
        ${OSG_GEO_OPENTHREADS_LIBRARY}
        ${OSG_GEO_OSGUTIL_LIBRARY}
        ${OSG_GEO_OSGVIEWER_LIBRARY}
        ${OSG_GEO_OSGDB_LIBRARY}
        osgGeo
    )
    add_dependencies(osgGeoExamples ${name})
endmacro()

add_example(horizons horizons.cpp)
add_example(textureplane textureplane.cpp)
