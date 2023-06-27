#!/bin/bash
# Read in some options -h, -a, -c
    # Shift, shift 2;
    # - `selected_flag=${1:---h}` defaulting?
# 1. in array mode, concatenate strings applied by the user
#   - Pass the array in and access as a reference
#   - exit if the array is empty
#   - Print date in formated string: %Y-%m-%d
# 2. Read a file, replace word "the" with "da" 
#   - Set where the TASK folder is. Default is this file
#   - Share <TASK> across two different functions.
#   - sed to replace word the with "da"
#   - print the base name of the file, using parameter expansion.
usage(){
    echo "This file is an example script that either prints a given array, or read, replace, or print a given file's all words "the" with "da"."
    echo "Usage: $0 (print_arr <str1>, <str2> ... | read_replace_print FILE_PATH)"
}

figure_out_filepath(){
    # You can set var elsewhere
    FILE_PATH=${2:-"$(realpath "$0")"}
}

read_replace_print(){
    # Quirk: if FILE_PATH is set, even if it's empty, ${FILE_PATH+x} evaluates to x.
    # -n evaluates whether the string is empty. Not excatly necessary, because BASH CHECKS FOR EMPTY STRINGS. if [[ 0 ]] evaluates to true, because 0 is a non-empty string
    if [ "${FILE_PATH+x}" ]; then
        # ## will remove the longest match with "*/", which means everything until "/"
        echo "File Name: ${FILE_PATH##*/}"
        echo "Or by using basename: $(basename $FILE_PATH)"
    fi
    if [[ ! -f $FILE_PATH ]]; then
        echo "${FILE_PATH} doesn't exist. Switching to current file"
        FILE_PATH="$(realpath "0")"
    fi

    # Can feed a text file (in the form of file descriptor) into the while loop
    while read -r line; do
        # sed "s/TO_REPLACE/REPLACE_TO/". One can use # as a delimeter, if your pattern contains /
        echo "$line" | sed "s/the/da/" 
    done < "$FILE_PATH"

    TEXT="adsf
    asdfads
    adsfa"

    # Here you can echo an output into the while loop
    # But you need to convert it to file descriptor using <(), i.e., process substitution
    # Command is run in a subshell. process substitution will make the output appear as a file
    while read -r line; do
        echo $line
    done < <(echo "$TEXT")
}

print_arr(){
    # # local is to create 
    # Quirk: an array is (1 2 3) instead of ARR=(1,2,3)
    # This element 1,2,3 is a single string
    local -n ARR_LOCAL_REF=$1
    # Quirk: == is for string comparisons
    if [[ ${#ARR_LOCAL_REF[@]} == 0 ]]; then
        echo "ARRAY is empty. Arr: ${ARR_LOCAL_REF[@]} "
        exit 1
    fi

    echo "array not empty: ${ARR_LOCAL_REF[@]}"
    # Quirk: Just ... don't use printf. 
    # 1. It doesn't expand args like echo does
    # 2. It repeats the formatted string
    printf "printing the same message as above: %s\n" "${ARR_LOCAL_REF[@]}"

    # Quirk: without (), += will append to the first element
    # So rather, you'll see (112 3)
    ARR_LOCAL_REF+=("1")
    ARR_LOCAL_REF+=("2")
    echo "after changing an element in the array, now the array looks like: ${ARR_LOCAL_REF[@]}"
    
    # printf "Printing all args one by one \n"
    # this is an experiment for $@ and shift 2 
    for arg in $@; do 
        echo "$1"
        shift 2
    done 

    echo "today: $(date +%Y-%m-%d)"
}

# NOTE parameter expansion is like ${VAR}, but with default :-, if unset or null, this will expand to default.
# ${#VAR} is the length of the variable
for OPTION in $@; do
    OPTION=${1:--h}
    case ${OPTION} in
        -a)
            printf "========================\n"
            # or declare -a ARR
            ARR=(1 2 3 4)
            # ARR=()
            print_arr ARR
            ;;
        -h)
            printf "========================\n"
            usage
            ;;
        -c)
            printf "========================\n"
            figure_out_filepath $@
            read_replace_print
            ;;
    esac
    # -a doesn't play well with declare
    shift 1
done

# Test the

