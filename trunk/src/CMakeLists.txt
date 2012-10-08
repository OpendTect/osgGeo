foreach( myFolder
	 osgGeo )
    add_subdirectory( ${myFolder} )
endforeach()

find_path(OSGGEO_INCLUDE_DIR osgGeo ${CMAKE_CURRENT_SOURCE_DIR})
