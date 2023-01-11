# Complete is a built-in function for autocomplete
# dothis is the command whose args are to be completed
# now, never, tomorrow are the options
# put in /etc/bash_completion.d/ and it will be sourced automatically
complete -W "now never tomorrow" dothis
complete -A directory dothisindir
