#!/bin/bash

#This program compiles rico_application, then the program is run

function usage(){
	echo "Usage: sudo ${0} [OPTION].."
	echo "The default is to run all three compilation options: object files, static library, and dynamic library"
	echo "-s Compile the project using object files and linking only"
}
APP_NAME="./rico_app"
LIB_SRC="./dll"
UTIL_SRC="./dll_util"

if [[ "${#}" -gt 0 ]]
then
	while getopts s OPTION
	do
		case ${OPTION} in
			s)
				OBJECT_ONLY='true'
				;;
			?)
				usage
				exit 1
				;;
		esac
	done
fi

echo "------------Task1: Object Files Approach"
gcc -c "${LIB_SRC}.c" -o "${LIB_SRC}.o"
gcc -c "${UTIL_SRC}.c" -o "${UTIL_SRC}.o"
gcc -g -c "${APP_NAME}.c" -o "${APP_NAME}.o"
gcc -g "${APP_NAME}.o" "${UTIL_SRC}.o" "${LIB_SRC}.o" -o "${APP_NAME}" 
eval ${APP_NAME}
rip *.o

echo "obj: ${OBJECT_ONLY}"
if [[ "${OBJECT_ONLY}" == 'true' ]]
then
	exit 0
fi

echo "------------Task2: Static Library Approach"
gcc -c "${LIB_SRC}.c" -o "${LIB_SRC}.o"
gcc -c "${UTIL_SRC}.c" -o "${UTIL_SRC}.o"
ar -rs librico.a "${LIB_SRC}.o" "${UTIL_SRC}.o" 
gcc -c "${APP_NAME}.c" -o "${APP_NAME}.o" 
gcc "${APP_NAME}.o" -o "${APP_NAME}" -L . -lrico
eval ${APP_NAME}
rip *.o *.a

echo "------------Task3: Dynamic  Library Approach"
gcc -c -fPIC "${LIB_SRC}.c" -o "${LIB_SRC}.o"
gcc -c -fPIC "${UTIL_SRC}.c" -o "${UTIL_SRC}.o"
gcc "${LIB_SRC}.o" "${UTIL_SRC}.o" -shared -o librico.so
gcc -c "${APP_NAME}.c" -o "${APP_NAME}.o"
if [[ ${UID} -ne 0 ]]
then
	echo "Please run this file with sudo privileges"
	rip librico.so
else
	cp librico.so /usr/lib 
	ldconfig
	gcc "${APP_NAME}.o" -o "${APP_NAME}" -lrico
	eval ${APP_NAME}
	rip /usr/lib/librico.so
	rip *.o
fi



