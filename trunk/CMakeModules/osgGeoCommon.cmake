# Sets up common variables
#	OSGGEO_LIB_POSTFIX
#	OSGGEO_MAKE_LAUNCHERS
#
# Written by: Kristofer Tingdahl


if ( UNIX )
    if ( APPLE )
	set ( CMAKE_CXX_FLAGS "-Wno-unused-parameter -Wno-unused -Wno-unused-function -Wno-unused-label -Wno-shadow -Wno-overloaded-virtual ${CMAKE_CXX_FLAGS}" ) #Sysroot does not do the job
    else() #Linux
	if ( GCC_VERSION VERSION_GREATER 4.2 )
            set ( CMAKE_CXX_FLAGS "-Wignored-qualifier ${CMAKE_CXX_FLAGS}" )
        endif()
    endif()

    set ( CMAKE_CXX_FLAGS "-Woverloaded-virtual -Wno-reorder ${CMAKE_CXX_FLAGS}" )
    set ( CMAKE_CXX_FLAGS "-Wunused -Wmissing-braces -Wparentheses -Wsequence-point ${CMAKE_CXX_FLAGS}" )
    set ( CMAKE_CXX_FLAGS "-Wswitch -Wunused-function -Wunused-label ${CMAKE_CXX_FLAGS}" )
    set ( CMAKE_CXX_FLAGS "-Wshadow -Wwrite-strings -Wpointer-arith -Winline ${CMAKE_CXX_FLAGS}" )
    set ( CMAKE_CXX_FLAGS "-Wformat -Wmissing-field-initializers ${CMAKE_CXX_FLAGS}" )
    set ( CMAKE_CXX_FLAGS "-Wreturn-type -Winit-self -Wno-char-subscripts ${CMAKE_CXX_FLAGS}" )


    set ( CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem ")
    set ( CMAKE_INCLUDE_SYSTEM_FLAG_C "-isystem ")
    
endif()

if ( WIN32 )
    add_definitions( "/W4" "/EHsc" "-DWIN32" "/wd4127" "/wd4800" )
endif( WIN32 )



if ( UNIX AND NOT WIN32 AND NOT APPLE )
  if (CMAKE_SIZEOF_VOID_P MATCHES "8")
      set( OSGGEO_LIB_POSTFIX "64" CACHE STRING "suffix for 32/64 dir placement")
      mark_as_advanced( OSGGEO_LIB_POSTFIX)
  endif()
else()
    set(OSGGEO_MAKE_LAUNCHERS 1)
endif()

if ( NOT DEFINED OSGGEO_LIB_POSTFIX)
    set(OSGGEO_LIB_POSTFIX "")
endif()
