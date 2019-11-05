find_package(Eigen3 REQUIRED)

if (EIGEN3_FOUND)
  message("Eigen3 FOUND: ${EIGEN3_INCLUDE_DIR}")
endif(EIGEN3_FOUND)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(problem4 problem5.cpp)

