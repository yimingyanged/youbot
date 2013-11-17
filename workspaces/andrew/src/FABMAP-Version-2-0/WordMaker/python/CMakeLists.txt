find_package( Boost REQUIRED python)
include_directories(${Boost_INCLUDE_DIRS})

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

#build a list of files to compile
SET(WordMakerPythonInterface_SRCS
  WordMakerPythonInterface.cpp
)

add_library(WordMakerPythonInterface ${WordMakerPythonInterface_SRCS})
target_link_libraries(WordMakerPythonInterface
  WordMaker 
  ${WordMaker_DEPEND_LIBS}
  ${PYTHON_LIBRARIES}
  ${PLATFORM_LIBS}
  ${Boost_LIBRARIES}
)

if ( WIN32 )
    set_target_properties(WordMakerPythonInterface PROPERTIES OUTPUT_NAME "wordmaker_python" SUFFIX ".pyd")
endif ( WIN32 )
