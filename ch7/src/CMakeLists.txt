set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -mpopcnt")
message(STATUS "-std=c++11")

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV FOUND: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV LIBS: ${OpenCV_LIBS}")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
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

# .so
add_library(orb_utils SHARED orb_utils.hpp orb_utils.cpp)