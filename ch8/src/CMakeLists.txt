add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV FOUND: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 FOUND: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Eigen3 libs: ${EIGEN3_LIBRARIES}")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow
    /usr/lib/liborb_utils.so
    ${OpenCV_LIBS}
)

add_executable(optical_flow_gaussion optical_flow_gaussion.cpp)
target_link_libraries(optical_flow_gaussion
    ${OpenCV_LIBS}
    /usr/lib/liborb_utils.so
    /usr/lib/libflow_utils.so
)

# flow_utils.so 
add_library(flow_utils SHARED flow_utils.hpp flow_utils.cpp)

set_target_properties(flow_utils PROPERTIES VERSION 1.0 SOVERSION 1)

install(TARGETS flow_utils
        LIBRARY DESTINATION /usr/lib 
        ARCHIVE DESTINATION /usr/lib
)

install(FILES ./flow_utils.hpp DESTINATION /usr/include)