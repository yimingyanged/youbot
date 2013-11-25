set(FABMAP_COMMON_SOURCES
IOHelperFunctions.cpp
MAT_parser.cpp
MJCMath.cpp
OXS_iterator.cpp
OXS_parser.cpp
OXS_parser_helper.cpp
)

add_library(FabMapCommon STATIC ${FABMAP_COMMON_SOURCES})
target_link_libraries(FabMapCommon MOOSGen vnl ${PLATFORM_LIBS})
