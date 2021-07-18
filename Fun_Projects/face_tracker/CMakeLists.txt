cmake_minimum_required(VERSION 3.10.0)	#telling CMake
project(face_tracker VERSION 1.0.0)

add_subdirectory(pybind11)
pybind11_add_module(face_tracker face_tracker.cpp)
target_include_directories(face_tracker PUBLIC include)

# add_library(filters STATIC particle_filter.cpp)
target_compile_definitions(face_tracker  VERSION_INFO=${EXAMPLE_VERSION_INFO})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++2a -O3") 
add_definitions(${GCC_COMPILE_FLAGS})

# target_include_directories(filters PUBLIC include)


