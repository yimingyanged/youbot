CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Bayesian Loop Closure
PROJECT(FabMap)

# Make sure we can find cmake scripts.  This line implies a dependency on MRG Core.
# If undesirable, we could always make a local cmake_modules directory.
SET( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CmakeModules )

# Put list of platform dependent libraries in PLATFORM_LIBS variable
include(SetPlatform)

# Make sure stuff goes in to the correct places in the build dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)

set(BUILD_SHARED_LIBS true)

##############################################################################
# Core library dependencies

find_package(ZLIB REQUIRED)
include_directories(${ZLIB_INCLUDE_DIRECTORIES})

find_package(MOOS REQUIRED) 
include_directories(${MOOS_INCLUDE_DIRS})
link_directories(${MOOS_LIBRARY_DIRS})

find_path(CImg_INCLUDE_DIR 
  CImg.h
  PATHS /usr/local/include
)
if (CImg_INCLUDE_DIR)
  message(STATUS "CImg_INCLUDE_DIR = ${CImg_INCLUDE_DIR}")
  include_directories(${CImg_INCLUDE_DIR})
else()
  message(ERROR "Please specify the path to the CImg.h parent directory.")
endif ()

find_package(VXL REQUIRED )
if (VXL_FOUND)
  include(${VXL_CMAKE_DIR}/UseVXL.cmake)
endif()

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

##############################################################################

add_subdirectory(ThirdParty)

# Library is common to both versions of FabMap
add_subdirectory(FabMapCommon)

# These should always build OK on all platforms
add_subdirectory(FabMapV2)

# Accelerated Chow Liu Tree Learner
add_subdirectory(pAcceleratedChowLiu_Fast)
add_subdirectory(pAcceleratedChowLiu_CompactMemory)

add_subdirectory(KMeans)

# You must select which variant of SURF to use in next step.
option(BUILD_WordMaker "Build WordMaker using abstract SURF Interface" OFF)
if (BUILD_WordMaker)
  add_subdirectory(WordMaker)
  if (BUILD_PythonInterfaces)    
    add_subdirectory(WordMakerPythonInterface)
  endif()
endif()


#option(BUILD_WordMakerOpenSURFAlone "Build WordMaker using OpenSURF" OFF)
#if (BUILD_WordMakerOpenSURFAlone)
#  add_subdirectory(WordMakerOpenSURF)
#  if (BUILD_PythonInterfaces)    
#    add_subdirectory(WordMakerPythonInterface)
#  endif()
#endif()

#option(BUILD_WordMakerAbstract "Build WordMaker using abstract SURF Interface" OFF)
#if (BUILD_WordMakerPanOmatic)
#  add_subdirectory(WordMakerPanOmatic)
#  if (BUILD_PythonInterfaces)
#    add_subdirectory(WordMakerPythonInterface)
#  endif()
#endif()

