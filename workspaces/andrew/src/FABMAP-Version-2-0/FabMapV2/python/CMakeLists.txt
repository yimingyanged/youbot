find_package( Boost REQUIRED python)
include_directories(${Boost_INCLUDE_DIRS})

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

#build a list of files to compile
SET(FabMapPythonInterface_SRCS
FabMapPythonInterface.cpp
)

ADD_LIBRARY(FabMapPythonInterface ${FabMapPythonInterface_SRCS})
TARGET_LINK_LIBRARIES(FabMapPythonInterface
${FabMapV2_DEPEND_LIBS}
FabMapV2
${PYTHON_LIBRARIES}
${Boost_LIBRARIES}
)

if ( WIN32 )
    set_target_properties(FabMapPythonInterface PROPERTIES OUTPUT_NAME "fabmap_python" SUFFIX ".pyd")
endif ( WIN32 )
