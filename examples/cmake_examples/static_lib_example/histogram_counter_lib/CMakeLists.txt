cmake_minimum_required(VERSION 3.0.2)
project(histogram_counter_lib)
set(PACKAGE_VERSION 1.0)

set(CMAKE_CXX_FLAG "-g -Wall -std=c++11")

add_library (histogram_counter_lib histogram_counter_lib.cpp)
target_include_directories(histogram_counter_lib PUBLIC "${PROJECT_SOURCE_DIR}/include")  #this is better than include_directories, because we MUST specify PUBLIC here. Otherwise, cuda_add_library seems to have include directory private. see https://stackoverflow.com/questions/41814426/cmake-cant-find-header-files

target_link_libraries(histogram_counter_lib pthread)
target_compile_options(histogram_counter_lib PUBLIC -fPIC -std=c++11)

