#!/bin/bash

# Generates a password. Bonus: the order of input arguments do not matter.  

usage(){
	echo "Usage: ${0} [-vs] [-l LENGTH]"
	echo "generate a random password"
	echo "-v Increase Verbosity"
	echo "-s Append a special character to the password"
	echo "-l LENGTH Specify the password length"
}

echo "getopt demo"
PASSWORD_LEN=48
while getopts 'vl:s' OPTION	#getopts are used more often, '' is optional, h: means argument is following. OPTION will store the current option, a built-in OPTARG will store the argument. Also, use a while loop to parse all arguments.  
do 
	case ${OPTION} in
		v)
			echo "VERBOSE"
			;;
		l)
			PASSWORD_LEN=${OPTARG}
			echo "now length is ${PASSWORD_LEN}"
			;;
		s)
			echo "SPECIAL CHAR"
			USE_SPECIAL_CHAR='true'
			;;
		?)	#? is for one single character. 
			usage
			# an error msg is already generated
			exit 1
			;;
	esac
done 

#generate a password
PASSWORD=$(date +%s%N${RANDOM} | shasum -a 1 | shasum -a 256 | head -c ${PASSWORD_LEN})
if [[ ${USE_SPECIAL_CHAR} == 'true' ]]
then
	SPECIAL_CHAR=$(echo "+%_*()&^$" | fold -w1 | shuf | head -c1 )
	echo "special char: ${SPECIAL_CHAR}"
fi
PASSWORD="${PASSWORD}${SPECIAL_CHAR}"
echo "Password: ${PASSWORD}"

#see the option index that has been read
echo "Index of the next option to read: ${OPTIND}"

#Demonstrate Arithmatics
echo "Arithmatic demo"
NUM1=4.0/5.0
function float_eval(){
	local FLOAT="$(echo "scale=2; $*" | bc -q 2>/dev/null)"
	echo ${FLOAT}
	return 0	#bash return can only work with int return codes
}
res=$(float_eval $NUM1)
echo "the result is ${res}"
NUM1=2
echo "now num is ${NUM1}"
(( NUM1++ ))
echo "now num is ${NUM1}"
