#build a list of files to compile
SET(FABMAPV2_SRCS
FabMap.cpp
InnerLoop.cpp
Sampler.cpp
RankingFunction.cpp
KeyframeDetector.cpp
FileIO.cpp
LocationPrior.cpp
BatchProcess.cpp
ExternalAPI.cpp
)

# Add library
add_library(FabMapV2 STATIC ${FABMAPV2_SRCS})
target_link_libraries(FabMapV2 ${FabMapV2_DEPEND_LIBS})

# Add executable
add_executable(pFabMapV2 FabMapMain.cpp)
target_link_libraries(pFabMapV2 FabMapV2 ${FabMapV2_DEPEND_LIBS} ${PLATFORM_LIBS})



