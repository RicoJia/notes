#!/bin/bash

#display first 3 params
echo "${1} ${2} ${3}"

#loop thru all positional params
echo "Using for loop"
for P in "${@}"
do
	echo $P
done

#use a while loop

X=0
while [[ ${X} -lt 3 ]]
do 
	echo "X: "$((X+=1))
	echo "arg 1: ${1} | arg 2: ${2}"
	shift
done
