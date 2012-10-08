# Sets up common variables
#	OSGGEO_LIB_POSTFIX
#	OSGGEO_MAKE_LAUNCHERS
#
# Written by: Kristofer Tingdahl


if (UNIX AND NOT WIN32 AND NOT APPLE)
  if (CMAKE_SIZEOF_VOID_P MATCHES "8")
      set(LIB_POSTFIX "64" CACHE STRING "suffix for 32/64 dir placement")
      mark_as_advanced(LIB_POSTFIX)
  endif()
else()
  set(OSGGEO_MAKE_LAUNCHERS 1)
endif()

if ( NOT DEFINED OSGGEO_LIB_POSTFIX)
    set(OSGGEO_LIB_POSTFIX "")
endif()
