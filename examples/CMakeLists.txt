include_directories(
    ${OSG_INCLUDE_DIR}
    ${OSGGEO_INCLUDE_DIR}
)

macro(add_example name)
    add_executable(${name} EXCLUDE_FROM_ALL ${ARGN})
    target_link_libraries(${name}
        ${OSG_LIBRARY}
        ${OPENTHREADS_LIBRARY}
        ${OSGUTIL_LIBRARY}
        ${OSGVIEWER_LIBRARY}
        osgGeo
    )
endmacro()

add_example(horizons horizons.cpp)
