cmake_minimum_required(VERSION 3.0.0)

set(VAR OFF)

set(VAR2 VAR)

if(${VAR2})
	message("If block executed")
else()
	message("Else block executed")
endif()
