cmake_minimum_required(VERSION 3.0.0)
project(Test_Module_Project VERSION 1.0.0)

#Note you need to provide the <path-to-module5-directory>

list(APPEND CMAKE_MODULE_PATH <path-to-module5-directory>)
include(my_module)