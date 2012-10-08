include_directories(
    ${OSG_INCLUDE_DIR}
    ${OSGGEO_INCLUDE_DIR}
)

add_custom_target(osgGeoExamples)

macro(add_example name)
    add_executable(${name} EXCLUDE_FROM_ALL ${ARGN})
    target_link_libraries(${name}
        ${OSG_LIBRARY}
        ${OPENTHREADS_LIBRARY}
        ${OSGUTIL_LIBRARY}
        ${OSGVIEWER_LIBRARY}
        osgGeo
    )
    add_dependencies(osgGeoExamples ${name})
endmacro()

add_example(horizons horizons.cpp)
