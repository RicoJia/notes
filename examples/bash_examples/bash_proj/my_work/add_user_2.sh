#!/bin/bash 

#Add user with a password

#usage
usage(){
	echo "Usage: sudo ${0} NAME" >/dev/stderr
}

#Ensure sudo priviledges
if [[ "${UID}" -ne 0 ]]
then
	echo "Please run as sudo or root." >&2
	exit 1
fi

#Checks if a name is provided
if [[ "${#}" -lt 1 ]]
then 
	usage
	exit 1
fi


#add user account
USER_NAME=${1}
useradd -m -s /bin/bash ${USER_NAME}
if [[ "${?}" -ne 0 ]]
then
	usage
	exit 1
fi

# generate a password
PASS_WORD=$(date +%s%N${RANDOM} | shasum -a 256 | head -c 5)
(echo "${PASS_WORD}"; echo "${PASS_WORD}") | passwd ${USER_NAME}
if [[ "${?}" -ne 0 ]]
then
	echo "Failed to add password, so failed to create ${USER_NAME}" 1>&2
	userdel ${USER_NAME}
	exit 1
else
	echo "Successfully added ${USER_NAME}, host machine: $(hostname)"
	echo "Successfully added password ${PASS_WORD}"
fi
