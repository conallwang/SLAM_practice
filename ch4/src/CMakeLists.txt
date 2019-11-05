# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0x)
  message(STATUS "Using flag -std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} has no c++11 support. Please use a different C++ compiler")
endif()

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 FOUND: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Eigen LIBRARY: ${EIGEN3_LIBRARIES}")

find_package(Pangolin 0.2 REQUIRED)
message(STATUS "Pangolin FOUND: ${Pangolin_INCLUDE_DIRS}")
message(STATUS "Pangolin libs: ${Pangolin_INCLUDE_LIBRARIES}")

# find_package(Sophus REQUIRED)
# message(STATUS "Sophus FOUND: ${Sophus_INCLUDE_DIRS}")
# message(STATUS "Sophus LIBRARY: ${Sophus_LIBRARIES}")

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
#   ${Sophus_INCLUDE_DIRS}
)

add_executable(sophus sophus.cpp)
# target_link_libraries(sophus
#   ${Sophus_LIBRARIES}
# )

add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError
  ${Pangolin_LIBRARIES}
)
