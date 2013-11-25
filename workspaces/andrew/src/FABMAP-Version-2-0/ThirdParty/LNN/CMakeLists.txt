set( LNNLIB_SRCS
  nn.cpp
  util.cpp
)

add_library( LNN STATIC ${LNNLIB_SRCS})

# Make variables available to the rest of the project
set(LNN_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "")
set(LNN_LIBRARIES LNN CACHE INTERNAL "")