include_directories(${LNN_INCLUDE_DIRS})
include_directories( ${PROJECT_SOURCE_DIR}/FabMapCommon/include )
include_directories( include )

# Required by CImg library
if(NOT WIN32)
  find_package(X11 REQUIRED)
  include_directories(${X11_INCLUDE_DIRS})
endif()

set(WordMaker_DEPEND_LIBS
  FabMapCommon
  ${ZLIB_LIBRARIES}
  ${LNN_LIBRARIES}
  ${VNLLIBS}
  ${MOOS_LIBRARIES}
  ${X11_LIBRARIES}
  MOOSGen
)

add_subdirectory(src)

# Python interface to WordMaker
option(BUILD_PythonInterfaces "Build Python Interfaces for WordMaker" OFF)
if(BUILD_WordMakerPythonInterfaces)
  add_subdirectory(python)
endif()