cmake_minimum_required(VERSION 3.16.3)

project(Assignment2)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Assignment2 main.cpp rasterizer.cpp Triangle.cpp)

link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(Assignment2 ${OpenCV_LIBS})
