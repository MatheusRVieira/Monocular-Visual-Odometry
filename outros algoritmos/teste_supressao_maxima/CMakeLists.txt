cmake_minimum_required(VERSION 3.1)
project(teste)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(teste teste.cpp)

#message(FATAL_ERROR ${OpenCV_LIBS})
target_link_libraries( teste ${OpenCV_LIBS} )
