# KMeans Clustering

include_directories( include )

find_package( Boost REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )

set( BUILD_KMEANS_WITH_EXTERNAL_MEMORY_CONTAINERS OFF CACHE BOOL 
  "Compile with external memory conatiners (requires STXXL)" )

if( BUILD_PKMEANS_WITH_EXTERNAL_MEMORY_CONTAINERS )
  find_package( STXXL REQUIRED )
  if( STXXL_FOUND )
    include_directories(${STXXL_INCLUDE_DIR})
    link_directories(${STXXL_LIBRARY_DIR})
  else()
    message( ERROR "STXXL NOT found - Please specify where to find STXXL.")
  endif()
endif()

include_directories( ${LNN_INCLUDE_DIRS} )

set( KMeans_DEPEND_LIBS 
  ${LNN_LIBRARIES} 
  ${MOOS_LIBRARIES} 
  MOOSGen )

add_subdirectory( src )

