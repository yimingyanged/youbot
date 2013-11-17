include_directories( ${PROJECT_SOURCE_DIR}/FabMapCommon/include )
include_directories( include )

set(FabMapV2_DEPEND_LIBS FabMapCommon ${ZLIB_LIBRARIES} ${MOOS_LIBRARIES})

add_subdirectory(src)
add_subdirectory(examples)

# Python interface to FabMap
option(BUILD_PythonInterfaces "Build Python Interfaces for FabMap" OFF)
if(BUILD_PythonInterfaces)
  add_subdirectory(python)
endif()
