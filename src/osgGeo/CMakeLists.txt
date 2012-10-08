SET(LIB_NAME osgGeo)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake ${CMAKE_CURRENT_SOURCE_DIR}/Config)


ADD_LIBRARY( ${LIB_NAME} SHARED
	    ${LIB_PUBLIC_HEADERS}
	    PolyLine.cpp
	    Horizon3D.cpp
	    LayeredTexture.cpp )

target_link_libraries(
	${LIB_NAME}
	${OSG_LIBRARY}
	${OPENTHREADS_LIBRARY}
	${OSGUTIL_LIBRARY} )