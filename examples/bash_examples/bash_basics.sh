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

    #9 test -f will see if a file exists
    test -f $TEST_FILE || echo "hello, file $TEST_FILE exits"
}
test_if

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

function test_str(){
    a='Hello'
    b='World'
    c="${a} ${b}"
    echo "${c}"
}

# 11. run a command as a certain user. But you need to invoke this script as the root first
#$(command); ${var}: variable
test_run_user(){
    runuser -u "$(whoami)" echo "lol"
    runuser -u rjia whoami
}

# 12 logger: without anything, it just goes into /var/sys/log. with -s, it prints as stderr as well. You can pass in however many strings to print here too
logger "hellollloo" "hola"

# 13 export a shell variable
test_variable_export(){
    test_shell_var=13
    export test_shell_var

    # set a shell variable if that var doesn't exist
    # ${RICO_VAR:-lol} will return lol if RICO_VAR doesn't exist.  Note, lol is already a string
    RICO_VAR=${RICO_VAR:-lol}
}

# 14 Test ls
test_ls(){
    # anything that starts with an o, or p.
    # [] is the beginning of the char set
    ls [op]*
}

# {} is called braces
test_brace_expansion(){
    echo abc
    # see bad, bed, bard
    echo b{a,e,ar}d
    # You can do this with letters or numbers as well
    echo {b..t}
}

# test_auto_complete(){
#     # You need to add a file to "source"
#     # but for simple string autocompletion (not args complete)
#     # see complete_script_to_source.bash
#
# }

test_trap(){
    # set a signal to trap for: when INT or TERM comes up, do echo
    trap 'echo "You hit ctrl-c!"' INT 
    # This works when you do kill PID (not kill -9)
    trap 'echo "You tried to kill me"' TERM
    sleep 10
}

test_and_or(){
    [ -z "$PS1" ] && echo "Skipping foyer because this terminal is non-interactive" || foyer
    # - $PS1 is an 'prompt variable', which is needed for prompting the user for an input. So it's set in an interactive shell session (not set in non-interactive shell). It looks like: ```\u@\h:\w$```, where u is username. h is homedirectory
    # - && and || will stop executing if the return value can be determined
}

test_stderr(){
    # This will directly redirect the output to stderr
    >&2 echo "Failed to load flags!"
    # This directs output to stdout, then stderr
    echo "Failed to load flags!" >&2 
}

test_loop(){
    # If a code is successful, sleep X always has exit value 0. In bash, any non-zero exit value is considered false, whereas 0 is considered true.  So while will take sleep 1; 
    # $? is the exit value of the last executed command
    # Then, break from the script
    while sleep 1; do echo "haha"; echo $? && break; done 

}
# test_loop


test_and(){
    # && will execute the latter command if the first statement is true.
    # { ...; ...; } executes a series of commands in a subshell; ( ...; ...; ) executes a series of commands in the current shell
    # CAVEAT: test -f <SPACE> still gives 0! so make sure $FILE is set!
    test -f $FILE && { echo "$FILE exists."; echo "hello!"; } && ( echo $FILE; echo "hellp again"; )
}
