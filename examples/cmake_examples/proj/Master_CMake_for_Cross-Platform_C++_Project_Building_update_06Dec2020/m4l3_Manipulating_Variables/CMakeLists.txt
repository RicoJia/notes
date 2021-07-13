cmake_minimum_required(VERSION 3.0.0)

set(NAME Alice)

set(Alice Bob)

set(NAMEAliceBob Charlie)

#message(${${NAME}})

#message(${NAME ${NAME} ${${NAME}}})

message(${NAME${NAME}${${NAME}}})

#message("NAME ${NAME} ${${NAME}}")

