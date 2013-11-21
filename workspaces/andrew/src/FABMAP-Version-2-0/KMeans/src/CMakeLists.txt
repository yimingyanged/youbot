set( KMeans_SRCS 
  mpi_kmeans.cxx
  approx_kmeans.cpp
  pKMeans.cpp
  ClusterInitializationMethods.cpp
  IOHelperFunctions.cpp
)

# Build a library
add_library( KMeans STATIC ${KMeans_SRCS} )
target_link_libraries( KMeans ${KMeans_DEPEND_LIBS} ${PLATFORM_LIBS} )

# Extra stuff for out of core containers
if( BUILD_PKMEANS_WITH_EXTERNAL_MEMORY_CONTAINERS )  
  target_link_libraries( KMeans ${STXXL_LIBRARIES} )
  
  add_definitions( 
    -DUSE_STXXL_OUT_OF_CORE_CONTAINERS
    -DSORT_OPT_PREFETCHING
    -DUSE_MALLOC_LOCK
    -DCOUNT_WAIT_TIME
    -D_FILE_OFFSET_BITS=64
    -D_LARGEFILE_SOURCE
    -D_LARGEFILE64_SOURCE
    )

  if(WIN32)
    add_definitions(
      -D_RTLDLL
      -DBOOST_LIB_DIAGNOSTIC
      -DSTXXL_BOOST_TIMESTAMP
      -DSTXXL_BOOST_CONFIG
      -DSTXXL_BOOST_FILESYSTEM
      -DSTXXL_BOOST_THREADS
      -DSTXXL_BOOST_RANDOM
      -D_SCL_SECURE_NO_DEPRECATE
      )
    
  endif()  
endif()


# Build an executable version
add_executable( pKMeans pKMeansMain.cpp )
target_link_libraries( pKMeans KMeans ${KMeans_DEPEND_LIBS} ${PLATFORM_LIBS} )

# Extra stuff for out of core containers
if( BUILD_PKMEANS_WITH_EXTERNAL_MEMORY_CONTAINERS )  
  target_link_libraries( pKMeans ${STXXL_LIBRARIES} )
  
  add_definitions( 
    -DUSE_STXXL_OUT_OF_CORE_CONTAINERS
    -DSORT_OPT_PREFETCHING
    -DUSE_MALLOC_LOCK
    -DCOUNT_WAIT_TIME
    -D_FILE_OFFSET_BITS=64
    -D_LARGEFILE_SOURCE
    -D_LARGEFILE64_SOURCE
    )

  if(WIN32)
    add_definitions(
      -D_RTLDLL
      -DBOOST_LIB_DIAGNOSTIC
      -DSTXXL_BOOST_TIMESTAMP
      -DSTXXL_BOOST_CONFIG
      -DSTXXL_BOOST_FILESYSTEM
      -DSTXXL_BOOST_THREADS
      -DSTXXL_BOOST_RANDOM
      -D_SCL_SECURE_NO_DEPRECATE
      )
    
  endif()  
endif()
