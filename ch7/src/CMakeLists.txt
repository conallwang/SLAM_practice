set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
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