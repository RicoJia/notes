#!/bin/bash
# 1. must require the shbang, otherwise, we might be running another shell
# function basics
NUM1=32
function float_eval(){
  local FLOAT="$(echo "scale=2; $*" | bc -q 2>/dev/null)"
  echo ${FLOAT}
}


# 2
function show_addition_and_func_name(){
    echo "${FUNCNAME[0]}" $((1+2))
}
# show_addition_and_func_name

# 3
function delete(){
    # note we have - instead of --. This method is not like xargs rm -rf, which has a max of 1000 something
    find $pwd -type f -name *log -delete
}
# delete

#4
function test_if(){
    # - need spacing after if, before [[. ]]. Spacing matters, else it won't evaluate properly!!
    # [[ is a **bash-only keyword. old way is [**. [[]] is an alias for test, a shell built-in command. So it's equivalent to having 3 arguments ```test ${VAR} == 'B'```
    # /newline is a command separate, same as ;**
    if [[ 1 ]]
    then 
        echo "hehe"
    fi

    # 5 - "" is used whenever the variable might contain space or is empty. This way, the entire thing is treated as a single argument. Else, the string might be broken into many. So, always wrap string with ""
    # 6 - UID is 0 if you execute the script as root, UID is a **shell built-in variable, not an environment variable**.  
    # 7 -eq for equal, = for comparing strings, == is for bash only. -ne is not equal. != for strings. -le is less than or equal to
    # 8 You also need space for -eq, so it is a separate word.
    if [[ ${UID} -eq 0 ]]; then echo "UID is 0"; fi
}
# test_if

# 8
function test_increase_variable(){
    I=1
    B=0
    # 9 Note that {a..b}, 3 dots, no extra spacing
    for i in {2..5}
    do
        # 10. integer math $(())
        I=$((I+1))
        ((B+=1))
    done 
    echo "I is ${I}, B is ${B}"
}
test_increase_variable

function test_str(){
    a='Hello'
    b='World'
    c="${a} ${b}"
    echo "${c}"
}
test_str

# 11. run a command as a certain user. But you need to invoke this script as the root first
#$(command); ${var}: variable
runuser -u "$(whoami)" echo "lol"
runuser -u rjia whoami

# 12 logger: without anything, it just goes into /var/sys/log. with -s, it prints as stderr as well. You can pass in however many strings to print here too
logger "hellollloo" "hola"

# 13 export a shell variable
test_shell_var=13
export test_shell_var

# set a shell variable if that var doesn't exist
# ${RICO_VAR:-lol} will return lol if RICO_VAR doesn't exist. 
RICO_VAR=${RICO_VAR:-lol}
