add_library(my_lib
	src/addition_lol.cpp
	src/division.cpp
	src/print_result.cpp
	)
target_include_directories(
	my_lib PUBLIC include
	)
