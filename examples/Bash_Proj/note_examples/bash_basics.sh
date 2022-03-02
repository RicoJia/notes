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
delete
