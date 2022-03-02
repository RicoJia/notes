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
