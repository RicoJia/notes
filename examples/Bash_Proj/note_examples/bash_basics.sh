#!/bin/bash
# must require the shbang, otherwise, we might be running another shell
# function basics
NUM1=32
function float_eval(){
  local FLOAT="$(echo "scale=2; $*" | bc -q 2>/dev/null)"
  echo ${FLOAT}
}

# res=$(float_eval $NUM1)
# echo "the result is ${res}"

function show_addition_and_func_name(){
    echo "${FUNCNAME[0]}" $((1+2))
}
# show_addition_and_func_name

function delete(){
    # note we have - instead of --. This method is not like xargs rm -rf, which has a max of 1000 something
    find $pwd -type f -name *log -delete
}
# delete

function test_if(){
    # - need spacing after if, before [[. ]]. Spacing matters, else it won't evaluate properly!!
    # [[ is a **bash-only keyword. old way is [**. [[]] is an alias for test, a shell built-in command. So it's equivalent to having 3 arguments ```test ${VAR} == 'B'```
    # /newline is a command separate, same as ;**
    if [[ 1 ]]
    then 
        echo "hehe"
    fi

    # - "" is used whenever the variable might contain space or is empty. This way, the entire thing is treated as a single argument. Else, the string might be broken into many. So, always wrap string with ""
    # - UID is 0 if you execute the script as root, UID is a **shell built-in variable, not an environment variable**.  
    # -eq for equal, = for comparing strings, == is for bash only. -ne is not equal. != for strings. -le is less than or equal to
    # You also need space for -eq, so it is a separate word.
    if [[ ${UID} -eq 0 ]]; then echo "UID is 0"; fi
}
test_if
