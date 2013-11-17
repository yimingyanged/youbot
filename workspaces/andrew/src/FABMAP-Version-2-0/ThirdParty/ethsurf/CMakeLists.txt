# The SURF library is supplied as binaries, so we can't do much with CMake,
# other than to provide a suitable config script, and (perhaps) copy the binaries into
# the appropriate build directory.


SET( SURF_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} )

# Currently expects to find the binaries already in the
# source tree.
SET( SURF_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR} )


# Choose correct version of SURF
IF(WIN32)
    SET(SURF_INCLUDE_DIRS ${SURF_SOURCE_DIR}/Win32)
    IF(MSVC80)
        SET(SURF_LIBRARY_DIR ${SURF_SOURCE_DIR}/Win32/Lib_VC8/)
    ELSE(MSVC80)
        SET(SURF_LIBRARY_DIR ${SURF_SOURCE_DIR}/Win32/Lib_VC7/)
    ENDIF(MSVC80)
    
    SET(SURF_LIBRARIES ${SURF_LIBRARY_DIR}/surfWINDLL)
ELSE(WIN32)
    IF( "${CMAKE_SYSTEM_NAME}" MATCHES "Darwin" )
#OS X
        SET(SURF_INCLUDE_DIRS ${SURF_SOURCE_DIR}/OSX)
        SET(SURF_LIBRARY_DIR ${SURF_SOURCE_DIR}/OSX)
        SET(SURF_LIBRARIES ${SURF_LIBRARY_DIR}/libSurfMac.a)
    ELSE( "${CMAKE_SYSTEM_NAME}" MATCHES "Darwin" )
#Linux
        SET(SURF_INCLUDE_DIRS ${SURF_SOURCE_DIR}/Linux)
        SET(SURF_LIBRARY_DIR ${SURF_SOURCE_DIR}/Linux)
        SET(SURF_LIBRARIES ${SURF_LIBRARY_DIR}/libSurf.a)
    ENDIF( "${CMAKE_SYSTEM_NAME}" MATCHES "Darwin" )
ENDIF(WIN32)

# Create MRGDevicesConfig.cmake file for outside builds
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/SURFConfig.cmake.in
   ${CMAKE_CURRENT_BINARY_DIR}/SURFConfig.cmake @ONLY)

INCLUDE_DIRECTORIES(${SURF_INCLUDE_DIRS})

SET(EXAMPLE_SOURCES
  OSX/main.cpp
)


ADD_EXECUTABLE(example ${EXAMPLE_SOURCES})
TARGET_LINK_LIBRARIES(example ${SURF_LIBRARIES})


# This makes an executable which outputs interest points in the 'standard' format required by
# the detector diagnostics programs here:
# http://www.robots.ox.ac.uk/~vgg/research/affine/evaluation.html

#SET(DETECTOR_SOURCES
#  OpenSURFcpp/src/output_to_file.cpp
#)
#ADD_EXECUTABLE(OpenSURF.ln  ${DETECTOR_SOURCES})
#TARGET_LINK_LIBRARIES(OpenSURF.ln  ${OpenCV21_LIBRARIES}  ${projname})
