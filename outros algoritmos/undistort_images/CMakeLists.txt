cmake_minimum_required(VERSION 3.1)
project(undistort)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(undistort undistort.cpp)

#message(FATAL_ERROR ${OpenCV_LIBS})
target_link_libraries( undistort ${OpenCV_LIBS} )
