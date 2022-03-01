cmake_minimum_required(VERSION 3.1)
project(sparse)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(sparse sparse.cpp)

#message(FATAL_ERROR ${OpenCV_LIBS})
target_link_libraries( sparse ${OpenCV_LIBS} )
