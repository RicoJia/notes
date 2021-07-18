cmake_minimum_required(VERSION 3.0.0)

macro(print_detail name_var)
	message("My name is ${name_var}")
	set(name_var abc)
	message("My name is ${name_var}")
endmacro()

print_detail(Charlie)
