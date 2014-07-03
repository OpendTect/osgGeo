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

    if ( GCC_VERSION VERSION_GREATER 4.0 )
	set ( CMAKE_CXX_FLAGS "-Wsequence-point -Wmissing-field-initializers ${CMAKE_CXX_FLAGS}" )
    endif()

    set ( CMAKE_CXX_FLAGS "-Woverloaded-virtual -Wno-reorder ${CMAKE_CXX_FLAGS}" )
    set ( CMAKE_CXX_FLAGS "-Wunused -Wmissing-braces -Wparentheses ${CMAKE_CXX_FLAGS}" )
    set ( CMAKE_CXX_FLAGS "-Wswitch -Wunused-function -Wunused-label ${CMAKE_CXX_FLAGS}" )
    set ( CMAKE_CXX_FLAGS "-Wshadow -Wwrite-strings -Wpointer-arith -Wno-inline ${CMAKE_CXX_FLAGS}" )
    set ( CMAKE_CXX_FLAGS "-Wformat ${CMAKE_CXX_FLAGS}" )
    set ( CMAKE_CXX_FLAGS "-Wreturn-type -Winit-self -Wno-char-subscripts ${CMAKE_CXX_FLAGS}" )

    if ( CMAKE_COMPILER_IS_GNUCC )
	set ( CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem ")
	set ( CMAKE_INCLUDE_SYSTEM_FLAG_C "-isystem ")
    endif()
    
endif()

if ( WIN32 )
   set( CMAKE_CXX_WARNING_LEVEL 4 )
   if( CMAKE_CXX_FLAGS MATCHES "/W[0-4]" )
     string( REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" )
   else( CMAKE_CXX_FLAGS MATCHES "/W[0-4]" )
     set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4" )
   endif( CMAKE_CXX_FLAGS MATCHES "/W[0-4]" )

   add_definitions( "/EHsc" "-DWIN32" "/wd4127" "/wd4800" )
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
