#!/bin/bash

#Logs and backs up this file in a designated area, then the back up file is removed. So effectively, this program does nothing but serves as a demonstration
 
function log(){
    if [[ "${GLOBAL_VERBOSE}" = 'true' ]]
    then
        echo "global GLOBAL_VERBOSE is true: ${@}"
    else
        local DUMMY_VAR="${1}"
        shift
        echo "GLOBAL_VERBOSE is false, 1st var: ${DUMMY_VAR}, 2nd var: ${2}"
    fi  
}

readonly GLOBAL_VERBOSE='false' #many ppl do it this way
log 'heh'

backup_files(){
    FILE=${1}
    if [[ -f ${FILE} ]]
    then
        local BACKUP_FILE="/var/tmp/$(basename ${FILE}).$(date +%F%N)" #use basename, and date %F%N to get the current date
        echo "${FILE} exists, now let's back it up in ${BACKUP_FILE}"
        cp -p ${FILE} ${BACKUP_FILE}    #preserves the date of modification
        rm ${BACKUP_FILE}
    else
        echo "${FILE} does not exist"
        return 1
    fi
}

backup_files "${0}"
backup_files "/non-exists"

