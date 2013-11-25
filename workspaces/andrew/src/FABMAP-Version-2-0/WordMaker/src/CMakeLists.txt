set(WORDMAKER_SRCS
  WordMaker.cpp
  WordMakerMain.cpp
  FileIO.cpp
  ExternalAPI.cpp  
)
# The common InterestPoint format.
# All SURF implementations adhere to interface defined in SurfToolsInterface.h
# INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/ThirdParty/SurfInterface/")

option(BUILD_WordMakerPanoSURF "Build WordMaker using Panomatic SURF" ON)
if (BUILD_WordMakerPanoSURF)
	set(WORDMAKER_SRCS 
	${WORDMAKER_SRCS}
	# extra sources required for Panomatic Surf
    ${PROJECT_SOURCE_DIR}/ThirdParty/panomaticsurf/Image.cpp
    ${PROJECT_SOURCE_DIR}/ThirdParty/panomaticsurf/KeyPointDescriptor.cpp
    ${PROJECT_SOURCE_DIR}/ThirdParty/panomaticsurf/KeyPointDetector.cpp
    ${PROJECT_SOURCE_DIR}/ThirdParty/panomaticsurf/MathStuff.cpp  
  )
     # include the correct SurfTools.h
     INCLUDE_DIRECTORIES(../include/SurfInterface/panomaticsurf)
     INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/ThirdParty/panomaticsurf )
     
endif()


# add all libraries and interface header for ETHZ surf.
option(BUILD_WordMakerETHZSurf "Build WordMaker using ETHZ SURF" OFF)
if (BUILD_WordMakerETHZSurf)
# Find the ETHZ Surf libraries
find_path(SURF_INCLUDE_DIR surf.h)
if (WIN32)
    find_library(SURF_LIBRARIES surfWINDLL ${SURF_INCLUDE_DIR})
else ()
    if ( "${CMAKE_SYSTEM_NAME}" MATCHES "Darwin" )
        #OS X
        find_library(SURF_LIBRARIES SurfMac ${SURF_INCLUDE_DIR})
    else ()
        #Linux
        find_library(SURF_LIBRARIES Surf ${SURF_INCLUDE_DIR})
    endif ()
endif ( WIN32 )

set(WordMaker_DEPEND_LIBS 
	${WordMaker_DEPEND_LIBS}
	${SURF_LIBRARIES}
)
# include the correct ETHZ SurfTools.h 
INCLUDE_DIRECTORIES( ../include/SurfInterface/ethsurf)
include_directories(${SURF_INCLUDE_DIR})

endif()

# Build a library version
ADD_LIBRARY(WordMaker ${WORDMAKER_SRCS})
TARGET_LINK_LIBRARIES(WordMaker ${WordMaker_DEPEND_LIBS} ${PLATFORM_LIBS})

# Build an executable version
ADD_EXECUTABLE(pWordMaker WordMakerMain.cpp)
TARGET_LINK_LIBRARIES(pWordMaker WordMaker ${WordMaker_DEPEND_LIBS} ${PLATFORM_LIBS})



