find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 FOUND: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Eigen3 libs: ${EIGEN3_LIBRARIES}")

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV FOUND: ${OPENCV_INCLUDE_DIR}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIR}
)

add_executable(curveFitting_GN curveFitting_GN.cpp)
target_link_libraries(curveFitting_GN
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)