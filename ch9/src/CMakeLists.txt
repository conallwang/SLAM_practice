SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

list(APPEND CMAKE_MODULE_PATH ../cmake_modules)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV FOUND: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 FOUND: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Eigen3 libs: ${EIGEN3_LIBRARIES}")

find_package(Ceres REQUIRED)
message(STATUS "Ceres FOUND: ${CERES_INCLUDE_DIRS}")
message(STATUS "Ceres libs: ${CERES_LIBRARIES}")

find_package(G2O REQUIRED)
message(STATUS "G2O FOUND: ${G2O_INCLUDE_DIRS}")
message(STATUS "G2O libs: ${G2O_LIBRARIES}")

find_package(CSparse REQUIRED)
message(STATUS "CSparse FOUND: ${CSPARSE_INCLUDE_DIR}")
message(STATUS "CSparse libs: ${CSPARSE_LIBRARY}")

SET(G2O_LIBS
        g2o_cli
        g2o_ext_freeglut_minimal
        g2o_simulator
        g2o_solver_slam2d_linear
        g2o_types_icp
        g2o_types_slam2d
        g2o_core
        g2o_interface
        g2o_solver_csparse
        g2o_types_sba
        g2o_types_slam3d
        g2o_csparse_extension
        g2o_opengl_helper
        g2o_solver_dense
        g2o_stuff
        g2o_types_sclam2d
        g2o_parser
        g2o_solver_pcg
        g2o_types_data
        g2o_types_sim3
        cxsparse
        cholmod )

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
)

add_executable(BA_ceres BA_ceres.cpp common.cpp common.h random.h rotation.h)
target_link_libraries(BA_ceres
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
)

add_executable(BA_g2o BA_g2o.cpp common.cpp common.h)
target_link_libraries(BA_g2o
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    ${G2O_LIBRARIES}
    ${CSPARSE_LIBRARY}
    ${G2O_LIBS}
)