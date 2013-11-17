FIND_PACKAGE( Boost REQUIRED COMPONENTS system thread date_time iostreams filesystem)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

FIND_PACKAGE( STXXL REQUIRED )
INCLUDE_DIRECTORIES(${STXXL_INCLUDE_DIR})
LINK_DIRECTORIES(${STXXL_LIBRARY_DIR})

include_directories( ${PROJECT_SOURCE_DIR}/FabMapCommon/include )
include_directories(include)

add_subdirectory(src)
