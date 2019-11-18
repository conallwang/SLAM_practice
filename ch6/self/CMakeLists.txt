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

list(APPEND CMAKE_MODULE_PATH ../cmake_modules)

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 FOUND: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Eigen3 libs: ${EIGEN3_LIBRARIES}")

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV FOUND: ${OPENCV_INCLUDE_DIR}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")

find_package(Ceres REQUIRED)
message(STATUS "Ceres FOUND: ${CERES_INCLUDE_DIRS}")
message(STATUS "Ceres libs: ${CERES_LIBRARIES}")

find_package(G2O REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIR}
    ${G20_INCLUDE_DIRS}
)

add_executable(curveFitting_GN curveFitting_GN.cpp)
target_link_libraries(curveFitting_GN
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(curveFitting_LM curveFitting_LM.cpp)
target_link_libraries(curveFitting_LM
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(curveFitting_ceres curveFitting_ceres.cpp)
target_link_libraries(curveFitting_ceres
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
)

add_executable(curveFitting_g2o curveFitting_g2o.cpp)
target_link_libraries(curveFitting_g2o
    ${OpenCV_LIBS}
    ${EIGEN3_INCLUDE_DIRS}
    /usr/local/lib/libg2o_core.so
    /usr/local/lib/libg2o_stuff.so
)