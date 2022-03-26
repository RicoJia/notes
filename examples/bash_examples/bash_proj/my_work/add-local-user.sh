#!/bin/bash

#This script creates an account and a random password for a user. The user will type in their name and password according to prompts. 


#Make sure the user gives at least one argument, else, show help manual
if [[ ${#} -lt 1 ]]
then
	echo "Usage: ${0} USER_NAME [USER_NAME]... "
	exit 1
fi

for NAME in "${@}"
do
	PASS_WORD=$(date +%s%N | sha1sum | sha256sum)
	echo "New Password: ${PASS_WORD}"
done

#generate password

# make sure the account has been created by checking the last 3 lines of /etc/passwd

#Enforces that it be executed with superuser (root) privileges.  If the script is not executed with superuser privileges it will not attempt to create a user and returns an exit status of 1.

if [[ "$UID" -ne 0 ]] 
then
	#how do you exit? 
	echo "Please execute the script as sudo"
	exit 1
fi

#Prompts the person who executed the script to enter the username (login), the name for person who will be using the account, and the initial password for the account.
echo "Please key in the intented username and password: "
read -p "Type in user name: " USERNAME
read -p "Type in your password: " PASSWORD
echo "Username: ${USERNAME}, Password: ${PASSWORD}"

#Creates a new user on the local system with the input provided by the user.
useradd -m -s /bin/bash ${USERNAME}

# Check to see if the useradd command succeeded.

# Set the password.
(echo ${PASSWORD}; echo ${PASSWORD}) | passwd ${USERNAME}
#passwd -e ${USERNAME}
# Check to see if the passwd command succeeded.
if [[ ${?} -ne 0 ]]
then
	echo 'The account has not been created'
	exit 1
fi


