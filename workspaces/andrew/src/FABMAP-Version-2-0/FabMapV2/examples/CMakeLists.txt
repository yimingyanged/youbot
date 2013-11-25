# Add executable
add_executable(FabMapV2LibExample main.cpp)
target_link_libraries(FabMapV2LibExample FabMapV2 ${FabMapV2_DEPEND_LIBS} ${PLATFORM_LIBS})



