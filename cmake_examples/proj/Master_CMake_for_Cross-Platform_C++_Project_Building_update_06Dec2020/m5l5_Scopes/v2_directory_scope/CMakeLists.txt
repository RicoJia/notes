cmake_minimum_required(VERSION 3.0.0)

set(Name Charlie)

message("Root directory:	Before adding subdirectory: ${Name}")
add_Subdirectory(demo_subdirectory_scope)
message("Root directory:	After adding subdirectory: ${Name}")
