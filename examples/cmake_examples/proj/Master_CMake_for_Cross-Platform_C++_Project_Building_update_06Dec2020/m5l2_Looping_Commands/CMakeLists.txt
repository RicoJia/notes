cmake_minimum_required(VERSION 3.0.0)


set(VAR a)
#set(VAR ${VAR}a)
#set(VAR ${VAR}a)
#set(VAR ${VAR}a)
#set(VAR ${VAR}a)
#set(VAR ${VAR}a)
#set(VAR ${VAR}a)
#set(VAR ${VAR}a)
#set(VAR ${VAR}a)
#set(VAR ${VAR}a)

while(NOT VAR STREQUAL "aaaaaaaaaa")
	set(VAR ${VAR}a)
	message(${VAR})
endwhile()


