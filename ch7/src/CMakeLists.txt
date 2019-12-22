set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -mpopcnt -lGL -lGLU -lglut")
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
    /home/johnson/SLAM/ch7/src/
)

add_executable(orb_cv orb_cv.cpp)
target_link_libraries(orb_cv
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)

add_executable(orb_self orb_self.cpp)
target_link_libraries(orb_self
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)

add_executable(orb_test orb_test.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(orb_test
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)

add_executable(2d2d 2d2d.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(2d2d
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)

add_executable(triangulation triangulation orb_utils.hpp orb_utils.cpp)
target_link_libraries(triangulation
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)

add_executable(opencv_PnP opencv_PnP.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(opencv_PnP
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
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
    ${Pangolin_LIBRARIES}
)

add_executable(BAGaussionNewton BAGaussionNewton.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(BAGaussionNewton
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    ${Pangolin_LIBRARIES}
)

add_executable(icp_svd icp_svd.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(icp_svd
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    ${Pangolin_LIBRARIES}
)

add_executable(icp_g2o icp_g2o.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(icp_g2o
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    ${G2O_LIB}
    ${Pangolin_LIBRARIES}
)

add_executable(icp_gaussion icp_gaussion.cpp orb_utils.hpp orb_utils.cpp)
target_link_libraries(icp_gaussion
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    ${Pangolin_LIBRARIES}
)

# .so
add_library(orb_utils SHARED orb_utils.hpp orb_utils.cpp)

set_target_properties(orb_utils PROPERTIES VERSION 1.0 SOVERSION 1)

install(TARGETS orb_utils
    LIBRARY DESTINATION /usr/lib
    ARCHIVE DESTINATION /usr/lib)

install(FILES ${PROJECT_SOURCE_DIR}/src/orb_utils.hpp DESTINATION /usr/include)