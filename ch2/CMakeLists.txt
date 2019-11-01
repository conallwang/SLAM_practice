cmake_minimum_required( VERSION 2.8 )
project(HelloSLAM)

# Debug Type
# set( CMAKE_BUILD_TYPE "DEBUG")

add_library(helloSLAMLIB SHARED helloSLAMLIB.cpp)
set_target_properties(helloSLAMLIB PROPERTIES OUTPUT_NAME helloSLAM)

add_executable(helloSLAM helloSLAM.cpp)
target_link_libraries(helloSLAM helloSLAMLIB)

install(TARGETS helloSLAM helloSLAMLIB
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib)
install(FILES helloSLAMLIB.h DESTINATION include/helloSLAM)
