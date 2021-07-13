cmake_minimum_required(VERSION 3.0.0)


function(print_detail name_var)
	message("My name is ${${name_var}}")

	if(DEFINED ARGV1)
		message("Hello, my name is ${ARGV1}")
	endif()

	message("ARGC=	${ARGC}")
	message("ARGV=	${ARGV}")
	message("ARGN=	${ARGN}")
	
	if(DEFINED ARGV0)
		message("ARGV0=	${ARGV0}")
	endif()
	if(DEFINED ARGV1)
		message("ARGV1=	${ARGV1}")
	endif()
	if(DEFINED ARGV2)
		message("ARGV2=	${ARGV2}")
	endif()
	if(DEFINED ARGV3)
		message("ARGV3=	${ARGV3}")
	endif()
endfunction()



set(Name Charlie)

print_detail(Name Bob Alice)
