# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  message(STATUS "Eigen3 Found: ${EIGEN3_INCLUDE_DIR}")
else(EIGEN3_FOUND)
  message(FATAL_ERROR "Could not find Eigen3!")
endif(EIGEN3_FOUND)

find_package(Pangolin REQUIRED)
message(STATUS "FOUND Pangolin Headers at: ${Pangolin_INCLUDE_DIRS}")
message(STATUS "FOUND Pangolin libs at: ${Pangolin_LIBRARIES}")
# if(PANGOLIN_FOUND)
#   message(STATUS "Pangolin Found: ${PANGOLIN_INCLUDE_DIR}")
#   message(STATUS "Pangolin libs: ${PANGOLIN_LIBRARY}")
# else(PANGOLIN_FOUND)
#   message(FATAL_ERROR "Could not find Pangolin!")
# endif(PANGOLIN_FOUND)

message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

add_executable(eigen_matrix eigen_matrix.cpp)
add_executable(eigen_geometry eigen_geometry.cpp)
add_executable(eigen_practice eigen_practice.cpp)
add_executable(eigen_practice_correct eigen_practice_correct.cpp)
add_executable(eigen_pangolin eigen_pangolin.cpp)

target_link_libraries(eigen_pangolin
  ${Pangolin_LIBRARIES}
)
