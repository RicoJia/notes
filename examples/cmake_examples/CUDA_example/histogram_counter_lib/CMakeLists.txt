cmake_minimum_required(VERSION 3.0.2)
project(histogram_counter_lib)
enable_language(CUDA)
set(PACKAGE_VERSION 1.0)

find_package(CUDA)

if (CUDA_FOUND)
    message("CUDA found, using device squaring!")
else()
    message("CUDA not found, doing something alternatively")
endif()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -gencode arch=compute_52,code=compute_52 -gencode arch=compute_62,code=compute_62)

set(CMAKE_CXX_FLAG "-g -Wall -std=c++11")

cuda_add_library(histogram_counter_lib  histogram_counter.cu)
target_include_directories(histogram_counter_lib PUBLIC "${PROJECT_SOURCE_DIR}/include")  #this is better than include_directories, because we MUST specify PUBLIC here. Otherwise, cuda_add_library seems to have include directory private. see https://stackoverflow.com/questions/41814426/cmake-cant-find-header-files

target_link_libraries(histogram_counter_lib pthread
    -ljpeg    #for libjpeg
  )
target_compile_options(histogram_counter_lib PUBLIC -fPIC -std=c++11)
set_target_properties(histogram_counter_lib
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


