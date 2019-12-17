set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -mpopcnt")
message(STATUS "-std=c++11")

list(APPEND CMAKE_MODULE_PATH ../cmake_modules)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV FOUND: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV LIBS: ${OpenCV_LIBS}")

find_package(Pangolin REQUIRED)
message(STATUS "Pangolin FOUND: ${Pangolin_INCLUDE_DIRS}")
message(STATUS "Pangolin libs: ${Pangolin_LIBRARIES}")

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 FOUND: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Eigen3 libs: ${EIGEN3_LIBRARIES}")

find_package(G2O REQUIRED)
message(STATUS "G2O FOUND: ${G2O_INCLUDE_DIRS}")
file(GLOB G2O_LIB /usr/local/lib/*g2o*.so)
message(STATUS "G2O libs: ${G2O_LIB}")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
)

add_executable(orb_cv orb_cv.cpp)
target_link_libraries(orb_cv
    ${OpenCV_LIBS}
)

add_executable(orb_self orb_self.cpp)
target_link_libraries(orb_self
    ${OpenCV_LIBS}
)

add_executable(orb_test orb_test.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(orb_test
    ${OpenCV_LIBS}
)

add_executable(test1 test.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(test1
    ${OpenCV_LIBS}
)

add_executable(2d2d 2d2d.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(2d2d
    ${OpenCV_LIBS}
)

add_executable(triangulation triangulation orb_utils.hpp orb_utils.cpp)
target_link_libraries(triangulation
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)

add_executable(opencv_PnP opencv_PnP.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(opencv_PnP
    ${OpenCV_LIBS}
)

add_executable(opencv_PnP_c opencv_PnP_c.cpp)
target_link_libraries(opencv_PnP_c
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    ${G2O_LIB}
)

add_executable(BAG2O BAG2O.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(BAG2O  
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    ${G2O_LIB}
)

add_executable(BAGaussionNewton BAGaussionNewton.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(BAGaussionNewton
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
)

# .so
add_library(orb_utils SHARED orb_utils.hpp orb_utils.cpp)