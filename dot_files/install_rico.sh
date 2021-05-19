#!/bin/bash

# Ensure Sudo Privileges
if [[ ${UID} -ne 0 ]]
then
	echo "Please run sudo ${0}"
	exit 1
fi

# vim-instant-markdown
apt install node
apt install npm
npm -g install instant-markdown-d
pip3 install --user smdv
curl -LO https://github.com/BurntSushi/ripgrep/releases/download/12.1.1/ripgrep_12.1.1_amd64.deb
dpkg -i ripgrep_12.1.1_amd64.deb



