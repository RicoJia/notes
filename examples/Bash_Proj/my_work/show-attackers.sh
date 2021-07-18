#!/bin/bash

# This program checks for login failures, and reports them in a csv file 

usage(){
	echo "This program checks for login failures, and reports them in a csv file "
	echo "Usage: ${0} LOG_FILE"
}

#Make sure more at least 1 argument is passed in
if [[ "${#}" -lt 1 ]]
then 
	usage 
	exit 1
fi

#Make sure log file exists
if [[ ! -f "${1}" ]]
then 
	echo "${1} does not exist!"
	exit 1
fi

#Read in input arguments
FILE=${1}
LIMIT=10
OUTPUT_FILE="show_attackers_log.csv"
echo "Count, IP, Location" >> ${OUTPUT_FILE}
grep Failed ${FILE} | awk -F 'from ' '{print $2}' | awk '{print $1}' | sort | uniq -c | sort -nr | while read FREQ IP_ADDRESS
do 
	if [[ ${FREQ} -gt ${LIMIT} ]]
	then 
		LOCATION=$(geoiplookup ${IP_ADDRESS} | awk -F ', ' '{print $2}')
		echo "${FREQ}, ${IP_ADDRESS}, ${LOCATION}" >> ${OUTPUT_FILE}
	fi
done 

