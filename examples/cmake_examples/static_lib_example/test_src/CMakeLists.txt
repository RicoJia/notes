cmake_minimum_required(VERSION 3.0.2)
project(histogram_counter)
set(PACKAGE_VERSION 1.0)

set(CMAKE_CXX_FLAG "-g -Wall -std=c++11")

set(COUNTER_DIR  ${PROJECT_SOURCE_DIR}/../histogram_counter_lib/include/)
set(COUNTER_LIB  histogram_counter_lib) #if you are to build histogram_counter_lib, you must use the name, not the future path to the libXX.a, ref: https://stackoverflow.com/questions/33599583/no-rule-to-make-target-with-internal-static-library
#?

message("RICO COUNTER_DIR: ${COUNTER_DIR}")
include_directories(COUNTER_DIR)
add_executable(histogram_counter histogram_counter.cpp)
# add_dependencies(histogram_counter ${COUNTER_LIB})
target_link_libraries(histogram_counter PUBLIC ${COUNTER_LIB})
target_compile_options(histogram_counter PUBLIC -std=c++14 -fPIC -O3 -Wno-unused-variable -Wno-unused-parameter -DOC_NEW_STYLE_INCLUDES)
SET_TARGET_PROPERTIES(histogram_counter PROPERTIES PREFIX "")

