cmake_minimum_required(VERSION 3.0.0)
project(DisplayImage)

find_package(OpenCV REQUIRED)

add_executable(DisplayImage main.cpp)

target_include_directories(DisplayImage PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(DisplayImage PRIVATE ${OpenCV_LIBS})

message("OpenCV Include Directories: "${OpenCV_INCLUDE_DIRS})
message("OpenCV Libraries: "${OpenCV_LIBS})
