	# - Find the Mission Oriented Operating Suite (MOOS) includes and libraries
#
# This module defines
#  MOOS_DIR          - A MOOS build directory.
#  MOOS_INCLUDE_DIRS - List of directories to find MOOS headers.
#  MOOS_LIBRARY_DIRS - Where to find MOOS libraries.
#  MOOS_LIBRARIES    - MOOS libraries to link with.
#  MOOS_FOUND        - If false, do not try to use MOOS.
#
# On windows, we check for MOOS build dirs in the registery.
#

SET(MOOS_DIR_DESCRIPTION "directory containing MOOS build - e.g /usr/local/share/MOOS or c:\\MOOS")

# Be quite if already in cache.
IF( MOOS_FOUND )
    SET( MOOS_FIND_QUIETLY TRUE )
ENDIF( MOOS_FOUND )

# First, try to find representative header.
FIND_PATH( MOOS_INCLUDE_INSTALL_DIR MOOSLIB/MOOSLib.h
	PATHS
    /usr/local/include
    /usr/include
    /opt/local/include
	ENV CPATH 
)
SET( MOOS_INCLUDE_INSTALL_DIR ${MOOS_INCLUDE_INSTALL_DIR} CACHE INTERNAL "" FORCE )

# Found header dir, from which we can deduce the library dir
IF( NOT "${MOOS_INCLUDE_INSTALL_DIR}" STREQUAL "MOOS_INCLUDE_INSTALL_DIR-NOTFOUND" )
    SET( MOOS_LIBRARIES MOOS MOOSGen MOOSUtility 
            CACHE STRING "MOOS libraries to link.")
	# Strip off the trailing "/include" in the path.
    GET_FILENAME_COMPONENT( LIB_SEARCH_PATH ${MOOS_INCLUDE_INSTALL_DIR} PATH )
    FIND_LIBRARY( MOOS_LIBRARY NAMES ${MOOS_LIBRARIES}
               PATHS ${LIB_SEARCH_PATH}/lib ENV CPATH FORCE )
    SET( MOOS_LIBRARY ${MOOS_LIBRARY} CACHE INTERNAL "" FORCE )

	IF( MOOS_LIBRARY )
		# Strip filename
		GET_FILENAME_COMPONENT( MOOS_LIBRARY_DIRS ${MOOS_LIBRARY} PATH  )
        SET( MOOS_LIBRARY_DIRS ${MOOS_LIBRARY_DIRS} CACHE PATH "" FORCE )
		SET( MOOS_INCLUDE_DIRS ${MOOS_INCLUDE_INSTALL_DIR} CACHE PATH "" FORCE )
	ENDIF( MOOS_LIBRARY )

	MARK_AS_ADVANCED( MOOS_LIBRARY_DIRS )
	MARK_AS_ADVANCED( MOOS_INCLUDE_DIRS )
	MARK_AS_ADVANCED( MOOS_LIBRARIES )
ENDIF( NOT "${MOOS_INCLUDE_INSTALL_DIR}" STREQUAL "MOOS_INCLUDE_INSTALL_DIR-NOTFOUND" )

# Couldn't find headers? ok, look for a build tree.
IF( "${MOOS_INCLUDE_INSTALL_DIR}" STREQUAL "MOOS_INCLUDE_INSTALL_DIR-NOTFOUND" )
	SET( MOOS_LIBRARIES MOOS MOOSGen MOOSUtility 
            CACHE STRING "MOOS libraries to link.")
			MARK_AS_ADVANCED( MOOS_LIBRARIES )   
			FIND_PATH( MOOS_DIR ${MOOS_DIR}/MOOSConfig.cmake

			# Read from the CMakeSetup registry entries.  It is likely that
			# MOOS will have been recently built.
			[HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild1]
			[HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild2]
			[HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild3]
			[HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild4]
			[HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild5]
			[HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild6]
			[HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild7]
			[HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild8]
			[HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild9]
			[HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild10]

			# Help the user find it if we cannot.
			DOC "The ${MOOS_DIR_DESCRIPTION}" )

	IF( NOT MOOS_DIR AND MOOS_FIND_REQUIRED )
		MESSAGE( FATAL_ERROR "Could NOT find MOOS build directory -- please provide a directory containing a MOOS build")
	ENDIF( NOT MOOS_DIR AND MOOS_FIND_REQUIRED )

	# get values form MOOSConfig.cmake
	INCLUDE( ${MOOS_DIR}/MOOSConfig.cmake )

    SET( MOOS_LIBRARY_DIRS
         ${MOOS_DIR}/MOOSBin ${MOOS_DIR}/lib
		 ${MOOS_DIR}/NavigationAndControl
		     CACHE STRING "List of MOOS library directories" FORCE )

	# this should work regardless
	SET( MOOS_INCLUDE_DIRS
         ${MOOS_SOURCE_DIR}/Core
         ${MOOS_SOURCE_DIR}/Essentials
         ${MOOS_SOURCE_DIR}/NavigationAndControl
         ${MOOS_SOURCE_DIR}/Instuments
         ${MOOS_SOURCE_DIR}/Tools
         ${MOOS_SOURCE_DIR}/Thirdparty
         CACHE STRING "List of MOOS include directories" FORCE )

ENDIF( "${MOOS_INCLUDE_INSTALL_DIR}" STREQUAL "MOOS_INCLUDE_INSTALL_DIR-NOTFOUND" )

IF( MOOS_INCLUDE_DIRS AND MOOS_LIBRARY_DIRS )
    SET( MOOS_FOUND TRUE CACHE INTERNAL "" )
ELSE( MOOS_INCLUDE_DIRS AND MOOS_LIBRARY_DIRS )
    SET( MOOS_FOUND FALSE CACHE INTERNAL "" )
ENDIF( MOOS_INCLUDE_DIRS AND MOOS_LIBRARY_DIRS )

IF( MOOS_FOUND )
    IF( NOT MOOS_FIND_QUIETLY )
#        MESSAGE( STATUS "Found MOOS headers: ${MOOS_INCLUDE_DIRS}" )
#        MESSAGE( STATUS "Found MOOS libraries: ${MOOS_LIBRARY_DIRS}" )
        MESSAGE( STATUS "Looking for MOOS headers -- found" )
        MESSAGE( STATUS "Looking for MOOS libraries -- found" )
    ENDIF( NOT MOOS_FIND_QUIETLY )
ELSE( MOOS_FOUND )
    IF( MOOS_FIND_REQUIRED )
        MESSAGE( FATAL_ERROR "Could NOT find MOOS -- please provide the directory containing MOOS.")
    ENDIF( MOOS_FIND_REQUIRED)
ENDIF( MOOS_FOUND )

MARK_AS_ADVANCED( MOOS_FOUND )
MARK_AS_ADVANCED( MOOS_INCLUDE_DIRS )
MARK_AS_ADVANCED( MOOS_LIBRARY_DIRS )
MARK_AS_ADVANCED( MOOS_LIBRARIES )
