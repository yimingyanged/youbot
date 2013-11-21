#
#  This Module sets the following variables
#
#  STXXL_INCLUDE_DIR  - Directory containing the stxxl headers
#  STXXL_LIBRARY_DIR  - Directory containing the stxxl libraries
#  STXXL_LIBRARIES    - Libraries to link against
#  STXXL_FOUND        - True when the STXXL include directory is found.
#
# MJC sez: This script tries to find STXXL library.  There is no default
# install location for STXXL, so this looks in some likely places.  If you put
# your library in a non-standard place, add a entry to the STXXL_DIR_SEARCH
# list.
#

# Be quite if already in cache.
IF( STXXL_FOUND )
    SET( STXXL_FIND_QUIETLY TRUE )
ENDIF( STXXL_FOUND )

SET(STXXL_INCLUDE_PATH_DESCRIPTION "directory containing the STXXL include files. E.g /usr/include/ or c:\\stxxl\\include")
SET(STXXL_DIR_MESSAGE "Set the STXXL_INCLUDE_DIR cmake cache entry to the ${STXXL_INCLUDE_PATH_DESCRIPTION}")

SET(STXXL_LIBRARY_DIR_PATH_DESCRIPTION "directory containing the STXXL library files. E.g /usr/lib or c:\\stxxl\\lib")
SET(STXXL_LIBRARY_DIR_MESSAGE "Set the STXXL_LIBRARY_DIR cmake cache entry to the ${STXXL_LIBRARY_DIR_PATH_DESCRIPTION}")

SET(STXXL_DIR_SEARCH $ENV{STXXL_ROOT})
IF(STXXL_DIR_SEARCH)
  FILE(TO_CMAKE_PATH ${STXXL_DIR_SEARCH} STXXL_DIR_SEARCH)
  SET(STXXL_DIR_SEARCH ${STXXL_DIR_SEARCH}/include)
ENDIF(STXXL_DIR_SEARCH)

#Some more places to look for STXXL.
# If you put your library in a non-standard place, add an entry here.
IF(WIN32)
  SET(STXXL_DIR_SEARCH
    ${STXXL_DIR_SEARCH}
    C:/
    D:/
    "C:/Program Files"
  )
ELSE(WIN32)
 SET( STXXL_DIR_SEARCH
    ${STXXL_DIR_SEARCH}
    $ENV{HOME}
    $ENV{PATH}
 )
ENDIF(WIN32)

# Add in some path suffixes. These will have to be updated whenever a new version comes out.
SET(SUFFIX_FOR_PATH
 stxxl
 stxxl-1.1.0
 stxxl-1.2.0
 stxxl-1.2.1
)


#Go look for STXXL
FIND_PATH(STXXL_DIR NAMES include/stxxl.h PATH_SUFFIXES ${SUFFIX_FOR_PATH} PATHS  ${STXXL_DIR_SEARCH}

  # Help the user find it if we cannot.
  DOC "The ${STXXL_INCLUDE_PATH_DESCRIPTION}"
)

# Now try and set the include and library paths
IF( STXXL_DIR )
    #Set the include dir
	SET( STXXL_INCLUDE_DIR ${STXXL_DIR}/include CACHE PATH "STXXL include directory" FORCE )
 	IF( NOT STXXL_FIND_QUIETLY )
 		MESSAGE(STATUS "Looking for STXXL -- found" )
	ENDIF( NOT STXXL_FIND_QUIETLY )

	IF( WIN32 )
		SET( STXXL_LIBRARIES libstxxl.lib CACHE STRING "" FORCE )
	ELSE( WIN32 )
		SET( STXXL_LIBRARIES libstxxl.a CACHE STRING "" FORCE )
	ENDIF( WIN32 )
	
    #Now find the libraries
    SET(STXXL_LIB_DIR_SEARCH ${STXXL_DIR} $ENV{PATH})
    FIND_PATH(STXXL_LIBRARY_DIR NAMES ${STXXL_LIBRARIES} PATH_SUFFIXES lib PATHS  ${STXXL_LIB_DIR_SEARCH}
      # Help the user find it if we cannot.
    DOC "The ${STXXL_LIBRARY_DIR_PATH_DESCRIPTION}" 
    )
 
	# make sure library was found
	IF( STXXL_LIBRARY_DIR )
		IF( NOT STXXL_FIND_QUIETLY )
			MESSAGE(STATUS "Looking for STXXL libraries -- found" )
		ENDIF( NOT STXXL_FIND_QUIETLY )
	ELSE( STXXL_LIBRARY_DIR )
		IF( STXXL_FIND_REQUIRED )
			MESSAGE(FATAL 
				"STXXL libraries were not found. ${STXXL_LIBRARY_DIR_MESSAGE}")
		ENDIF( STXXL_FIND_REQUIRED )
	ENDIF( STXXL_LIBRARY_DIR )
ELSE( STXXL_DIR )
	IF( STXXL_FIND_REQUIRED )
		MESSAGE("STXXL headers not found. ${STXXL_INCLUDE_DIR_MESSAGE}")
		MESSAGE("STXXL not found.  Please specify the STXXL source/build directory.")
	ENDIF( STXXL_FIND_REQUIRED )
ENDIF( STXXL_DIR )

IF( STXXL_INCLUDE_DIR AND STXXL_LIBRARY_DIR )
    SET( STXXL_FOUND TRUE CACHE INTERNAL "" )
ELSE( STXXL_INCLUDE_DIR AND STXXL_LIBRARY_DIR )
    SET( STXXL_FOUND FALSE CACHE INTERNAL "" )
ENDIF( STXXL_INCLUDE_DIR AND STXXL_LIBRARY_DIR )

MARK_AS_ADVANCED( STXXL_INCLUDE_DIR STXXL_LIBRARY_DIR STXXL_LIBRARIES )