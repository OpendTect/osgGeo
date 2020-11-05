# - List filtering functions
#
#  list_filter(var regex listitems...) - where var is the name of
#   your desired output variable, regex is the regex whose matching items
#   WILL be put in the output variable, and everything else is considered
#   a list item to be filtered.
#
#  list_filter_out(var regex listitems...) - where var is the name of
#   your desired output variable, regex is the regex whose matching items
#   will NOT be put in the output variable, and everything else is considered
#   a list item to be filtered.
#
# Original Author:
# 2009-2010 Ryan Pavlik <rpavlik@iastate.edu> <abiryan@ryand.net>
# http://academic.cleardefinition.com
# Iowa State University HCI Graduate Program/VRAC
#
# Copyright Iowa State University 2009-2010.
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)

if(__list_filter_out)
	return()
endif()
set(__list_filter_out YES)

function(list_filter_out var regex)
	set(_out)
	foreach(_item ${ARGN})
		set(_re)
		string(REGEX MATCH "${regex}" _re "${_item}")
		if(NOT _re)
			list(APPEND _out "${_item}")
		endif()
	endforeach()
	set(${var} "${_out}" PARENT_SCOPE)
endfunction()

function(list_filter var regex)
	set(_out)
	foreach(_item ${ARGN})
		set(_re)
		string(REGEX MATCH "${regex}" _re "${_item}")
		if(_re)
			list(APPEND _out "${_item}")
		endif()
	endforeach()
	set(${var} "${_out}" PARENT_SCOPE)
endfunction()
#Takes a list with both optimized and debug libraries, and removes one of them
#According to BUILD_TYPE

macro ( OD_FILTER_LIBRARIES INPUTLIST BUILD_TYPE )
    unset( OUTPUT )
    foreach ( LISTITEM ${${INPUTLIST}} )
	if ( DEFINED USENEXT )
	    if ( USENEXT STREQUAL "yes" )
		list ( APPEND OUTPUT ${LISTITEM} )
	    endif()
	    unset( USENEXT )
	else()
	    if ( LISTITEM STREQUAL "debug" )
		if ( "${BUILD_TYPE}" STREQUAL "Debug" )
		    set ( USENEXT "yes" )
		else()
		    set ( USENEXT "no" )
		endif()
	    else()
		if ( LISTITEM STREQUAL "optimized" )
		    if ( "${BUILD_TYPE}" STREQUAL "Release" )
			set ( USENEXT "yes" )
		    else()
			set ( USENEXT "no" )
		    endif()
		else()
		    list ( APPEND OUTPUT ${LISTITEM} )
		endif()
	    endif()
	endif()
    endforeach()

    set ( ${INPUTLIST} ${OUTPUT} )
endmacro()
