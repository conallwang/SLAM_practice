add_executable(helloSLAM main.cpp)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(SLAM REQUIRED)
if(SLAM_FOUND)
  include_directories(${SLAM_INCLUDE_DIR})
  target_link_libraries(helloSLAM ${SLAM_LIBRARY})
endif(SLAM_FOUND)
