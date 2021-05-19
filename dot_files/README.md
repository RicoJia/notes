## Vim 
### Coc extensions
- ccls: Follow [Coc installation guide ](https://codevion.github.io/#!vim/coc.md) for guides installing Coc
	- Install ccls
		- sudo apt install clang
		- [upgrade nodejs](https://phoenixnap.com/kb/update-node-js-version)
		- sudo apt-get install zlib1g-dev
		- [follow the build guide, note there's notes for ubuntu.](https://github.com/MaskRay/ccls/wiki/Build)
		- sudo cmake --build Release --target install
		- in vim, do :LspInstall... to install clangd. 
	- Coc Features
	- gd, gy to navigate to other files
	- shift +k for class definition, works with // and /\*\*/!
	- if you mess up a typo, esc and it will show error
	- select inside curly braces ```vi}```, but coc can allow you to do ```vif``` to select a function. 

- Install other coc packages
	- coc-bash: 
		- It's a known issue that starting vim from root might cause trouble
		```
		sudo npm i -g bash-language-server
		:CocInstall coc-sh
		copy to coc-settings.json (in ~/.vim) from https://github.com/neoclide/coc.nvim/wiki/Language-servers
		```
	- coc-docker
		```
		sudo npm install -g dockerfile-language-server-nodejs
		copy to coc-settings.json (in ~/.vim) from https://github.com/neoclide/coc.nvim/wiki/Language-servers 
		```
	- coc-yaml
		```:CocInstall coc-yaml```
	- coc-xml
		```
		:CocInstall coc-xml
		download java: https://www.digitalocean.com/community/tutorials/how-to-install-java-with-apt-on-ubuntu-18-04
		```
	- coc-vim
