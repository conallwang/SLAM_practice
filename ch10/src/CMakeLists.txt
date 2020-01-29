SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(CMAKE_MODULE_PATH APPEND "../cmake_modules")

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 FOUND: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Eigen3 libs: ${EIGEN3_LIBRARIES}")

find_package(G2O REQUIRED)
message(STATUS "G2O FOUND: ${G2O_INCLUDE_DIRS}")
message(STATUS "G2O libs: ${G2O_LIBRARIES}")

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
    ${G2O_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pose_gragh_g2o pose_gragh_g2o.cpp)
target_link_libraries(pose_gragh_g2o
    ${G2O_LIBRARIES}
    ${G2O_LIBS}
)

add_executable(pose_gragh_g2o_lie pose_gragh_g2o_lie.cpp)
target_link_libraries(pose_gragh_g2o_lie
    ${G2O_LIBRARIES}
    ${G2O_LIBS}
)
