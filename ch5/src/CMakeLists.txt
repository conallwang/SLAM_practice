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

# error
# add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV FOUND: ${OPENCV_INCLUDE_DIR}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")

include_directories(
#  ${OPENCV_INCLUDE_DIRS}
)

add_executable(opencv_base opencv_base.cpp)
target_link_libraries(opencv_base
  ${OpenCV_LIBS}
)

add_executable(opencv_undistorted opencv_undistort.cpp)
target_link_libraries(opencv_undistorted
  ${OpenCV_LIBS}
)

add_executable(opencv_undistorted_correct opencv_undistort_correct.cpp)
target_link_libraries(opencv_undistorted_correct
  ${OpenCV_LIBS}
)
