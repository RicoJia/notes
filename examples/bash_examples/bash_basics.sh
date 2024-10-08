#!/bin/bash
# General Notes
# 1. timer: you should check systemctl list-timers
# systemctl enable XX.timer, systemctl start XX.timer

# 1. must require the shbang, otherwise, we might be running another shell
# function basics
NUM1=32
FLOAT=12
function float_eval(){
  local FLOAT="$(echo "scale=2; $*" | bc -q 2>/dev/null)"
  echo ${FLOAT}
}
float_eval

# 2
# Note: echo can show multiple things
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
    
    if [[ 0 ]]
    then 
        echo "0 is evaluated as a string and since it's not empty,  it is also true"
    fi

    if [[ 1 ]]
    then 
        echo "1 is a non empty string so it is also true"
    fi

    if [[ "" ]]
    then
        echo "Empty string in the test construct is true"
    else
        echo "Empty string in the test construct is false"
    fi

    if [[ 1 -eq 1 ]]
    then 
        echo "1 -eq 1 is a numerical comparison, where 1 is evaluated as a number"
    fi

    function return_0(){
        return 0
    }

    function return_1(){
        return 1
    }

    if [[ $(return_1) ]]
    then 
        echo "Returning 1 in test construct is true"
    else
        echo "Returning 1 in test construct with $() is false"
    fi

    if return_0 
    then 
        echo "Returning 0 is also true"
    fi

    if return_1 
    then 
        echo "Returning 1 is also true"
    else
        echo "Returning 1 is NOT true"
    fi

}
test_if

function test_eid(){
    # 5 - "" is used whenever the variable might contain space or is empty. This way, the entire thing is treated as a single argument. Else, the string might be broken into many. So, always wrap string with ""
    # 6 - UID is 0 if you execute the script as root, UID is a **shell built-in variable, not an environment variable**.  
    # 7 -eq for equal, = for comparing strings, == is for bash only. -ne is not equal. != for strings. -le is less than or equal to
    # 8 You also need space for -eq, so it is a separate word.
    if [[ ${UID} -eq 0 ]]; then echo "UID is 0. The actual user is sudo"; fi
    # EUID is "effective UID", which is used for SUDO running access priviledge. 
    # EUID starts off with the UID value, but later it can be changed with setuid
    if [[ ${EUID} -ne 0 ]]; then echo "Not running effectively as sudo"; fi

    #9 test -f will see if a file exists
    test -f $TEST_FILE || echo "hello, file $TEST_FILE exits"
}

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

test_date(){
    # -I will make it ISO8601, n will show the full date
    date -In
}
# test_date

test_grep(){
    # grep -v is to not show lines with a certain word.. 
    # This line is common for filtering out greps
    grep -v grep
}
# test_grep

test_ping_and_disown(){
    # -c 2: sends 2 ICMP echo requests
    # ICMP is a protocol different from TCP/UDP, used to report, not correct
    # Network problems. DoS (denial of service) attack could be a flood of ICMP
    # Requests
    # -W 0.5: waits for a ping response with timeout=0.5
    ping 172.1.1.0 -c 2 -W 1.5 &
    # disown: Shell has an active_job_table, for running, suspended, background
    # disown -h marks all jobs on the active_job_table not to receive SIGHUP.
    # Then these processes are not reported by built-ins like job. This is great for long-running processes launched in SSH: it doesn't listen to SIGHUP signals for termination anymore. That means, when you close the SSH console, the process won't be terminated.
    # So, disown will let go of the ping background process. 
    disown -h
}
test_ping_and_disown

test_bash_source(){
    # : gives full path of script
    # Say this file sources B.sh. Then in B.sh: $0 gives this file, but BASH_SOURCE will give the entire list of files
    echo "testing bash source" ${BASH_SOURCE[0]}
}
test_bash_source

test_ssh(){
    # -t is psuedo terminal, which is required for password prompts
    ssh -t IP
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
    # while sleep 1; do echo "haha"; echo $? && break; done 

    # reading a file in bash: 1. Use read, which needs a file descriptor. 2. get the file descriptor using process substitution
    # Process substitution gives you the file descriptor of the command
    while read line; do echo "$line"; done < "lol"

}
test_loop


test_and(){
    # && will execute the latter command if the first statement is true.
    # { ...; ...; } executes a series of commands in a subshell; ( ...; ...; ) executes a series of commands in the current shell
    # CAVEAT: test -f <SPACE> still gives 0! so make sure $FILE is set!
    test -f $FILE && { echo "$FILE exists."; echo "hello!"; } && ( echo $FILE; echo "hellp again"; )
}

test_array(){
    echo "all inputs into the func: $@"
    # you can declare arrays like this as well.
    declare -a array1 array2
    shift 2
    echo "all inputs into the func after shifting 2 initial args: $@"
}
# test_array 1 2 3

test_test(){
    # in bash, if a is unset, you will see a difference. Else, if a is 0, it's still set
    a=0
    if [[ $a ]]; then echo "a exists"; else echo "a doesn't exist"; fi
}
test_test

test_listening_to_2_ros_topics(){
    # Interesting way to listen to two topics at once
    rostopic echo /rico & rostopic echo /george
}

test_xclip(){
    # - does ctrl-v a file
    xclip -sel clip file-name
}

test_jq(){
    # .name` will spit out "john"
    `echo '{"name":"John","age":30}' | jq 
}
