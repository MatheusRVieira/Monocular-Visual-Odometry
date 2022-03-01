cmake_minimum_required(VERSION 3.1)
project(dense)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(dense dense.cpp)

#message(FATAL_ERROR ${OpenCV_LIBS})
target_link_libraries( dense ${OpenCV_LIBS} )
