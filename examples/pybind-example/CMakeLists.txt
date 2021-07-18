cmake_minimum_required(VERSION 2.8.12)
project(example)

find_package(PythonInterp REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

add_subdirectory(pybind11)
pybind11_add_module(example example.cpp)

add_custom_command(TARGET PUBLIC example  POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/example.cpython* ${CMAKE_BINARY_DIR}/example.so)

