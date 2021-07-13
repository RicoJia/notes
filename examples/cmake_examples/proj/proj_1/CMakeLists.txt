cmake_minimum_required(VERSION 3.0.0)	#telling CMake
project(CALCULATOR VERSION 1.0.0)
add_subdirectory(my_lib)
#define RICO_OPTION for main.cpp
# set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -DRICO_OPTION")
add_executable(calculator 
main.cpp
)
target_link_libraries(calculator
	my_lib
)
