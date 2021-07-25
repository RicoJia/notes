cmake_minimum_required(VERSION 3.10.0)	#telling CMake
project(filters_test VERSION 1.0.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
add_definitions(${GCC_COMPILE_FLAGS})

message("rico: ${PROJECT_SOURCE_DIR}")
add_executable(filter_test ${PROJECT_SOURCE_DIR}/particle_filter_test.cc)   #must be in the same directory, else cmake seems to have difficulty finding it
include_directories(../include/)
target_link_libraries(filter_test gtest pthread )  #must have pthread no matter what your project is!!, also link the binary to the lib.
