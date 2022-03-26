#!/bin/bash

FILE="/tmp/data"
head -n2 /etc/passwd > ${FILE}

#redirects 1 line of the File to Echo
read LINE < ${FILE}
echo "1. printing 1 line: ${LINE}"

#redirect STDOUT to a file and overwrites
head -n3 /etc/passwd > ${FILE}
echo "2. Now printing file ${FILE}:"
cat ${FILE}

#Redirects STDOUT to a file and appends
echo ${LINE} >> ${FILE}
echo "3. Now printing file ${FILE}"
cat ${FILE}

#Redirects STDIN to program
read LINE 0< ${FILE}
echo "LINE contains: ${LINE}"

#Redirects STDOUT to a file using FD1, overwriting the file
head -n2 /etc/passwd 1> ${FILE}
printf "\n4.ERROR STREAM in ${FILE}"
cat ${FILE}	#cat does not place any errors in the file

#Redirects STDOUT and STDERR to a file, using 2>&1 
head -n2 /etc/passwd /error > ${FILE} 2>&1
printf "\n5.STDOUT and STDERR STREAM in ${FILE}"
cat ${FILE}	#cat does not place any errors in the file

#Redirects STDOUT and STDERR to a program, using pipe
printf "\n6.STDOUT and STDERR STREAM in ${FILE}"
head -n2 /etc/passwd /error |& cat

#Discard STDERR in blackhole
head -n2 /etc/passwd /error 1>/tmp/data 2>/dev/null
printf "\n7. STDERR discarded"
cat ${FILE}


