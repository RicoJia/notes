cmake_minimum_required(VERSION 3.0.0)

project(GTK_FindModule)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)
find_package(GTK3 REQUIRED)

if(NOT GTK3_FOUND)
	message(FATAL_ERROR "GTK3 not found")
endif()

add_executable(${PROJECT_NAME}_app  main.cpp)

target_include_directories(${PROJECT_NAME}_app  PRIVATE  ${GTK3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_app  PRIVATE  ${GTK3_LIBRARIES})
