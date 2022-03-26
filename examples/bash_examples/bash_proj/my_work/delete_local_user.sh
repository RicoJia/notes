#!/bin/bash

#deletes a user

# usage should be in stderr
usage(){
	echo "Usage: sudo ${0} [OPTION]... USERNAME1..." >&2
	echo "By default, we try to disable the account instead of deleting it."  >&2
	echo "Options: "
	echo "	-d Deletes an account instead of disabling it" >&2
	echo "	-r Removes the home directory associated with the account(s)" >&2
	echo "	-a Creates an archive associated with the account(s) in .tgz, then it will be stored in /archive/USERNAME"
}

#Make sure running as sudo
if [[ "${UID}" -ne 0 ]] || [[ ${#} -lt 1 ]]
then 
	usage	
	exit 1
fi

# Set flags
while getopts 'dra' OPTION
do 
	case ${OPTION} in 
		d) DELETE='true' ;;
		r) REMOVE='true' ;;
		a) ARCHIVE='true' ;;
		?) usage; exit 1 ;;
	esac
done 

shift $((OPTIND - 1))

#iterate thru all users
for USER_NAME in "$@"
do
	echo "User name: ${USER_NAME}"
	# Refuses any accounts with UID <= 1000
	if [[ "$(id -u ${USER_NAME})" -le 1000 ]]
	then 
		if [[ "${?}" -ne 0 ]]
		then
			echo "Failed to delete ${USER_NAME} as it has UID less than 1000."
		fi
		continue
	fi

	# Disable or delete the account
	if [[ ${DELETE} == 'true' ]]
	then
		userdel ${USER_NAME}
		echo "Deleted ${USER_NAME}"
	else
		chage -E -1 ${USER_NAME}
		echo "Disabled ${USER_NAME}"
	fi

	# Archive home directory. If Home directory doesn't exist, abort. 
	ARCHIVE_DIR="/archive/${USER_NAME}"
	if [[ ${ARCHIVE} == 'true' ]]
	then 
		echo "Archiving for user ${USER_NAME}"
		HOME_DIR="/home/${USER_NAME}"
		ARCHIVE_FILE="${USER_NAME}.tgz"
		if [[ ! -d ${HOME_DIR} ]]
		then 
			echo "${HOME_DIR} does not exist. Archiving Aborted"
			continue
		fi
		tar -Pzcf "${ARCHIVE_FILE}" "${HOME_DIR}" 	#This might fail as HOME_DIR might not exist
		#tar -zcf ${ARCHIVE_FILE} ${HOME_DIR} >&/dev/null	#This might fail as HOME_DIR might not exist
		echo "TODO: exit status: ${?}"
		if [[ "${?}" -ne 0 ]]
		then
			echo "Failed to create archive for ${USER_NAME}"
		else
			mkdir -p ${ARCHIVE_DIR}
			mv ${ARCHIVE_FILE} ${ARCHIVE_DIR}
			echo "Successfully archived ${USER_NAME}"
		fi
	fi	

	# Remove the home directory
	if [[ ${REMOVE} == 'true' ]]
	then
		if ! [[ -e "/home/${USER_NAME}" ]]
		then 
			echo "/home/${USER_NAME} does not exist. Failed to remove home directory." >&2
		else
			#rip "/home/{USER_NAME}"
			echo "/home/${USER_NAME} has been removed"
		fi
	fi

done

exit 0
