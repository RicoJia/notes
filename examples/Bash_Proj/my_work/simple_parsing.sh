#!/bin/bash

#case
case ${1} in  
  start)
      echo 'starting'
      ;;  
  stop)
      echo 'stop'
      ;;  
  status|--statu*)
      echo 'show status'
      ;;  
  *)  
      echo "Supply a valid option pls" >&2  #>&2 is to send stuff to STDERR
#     exit 1
      ;;  
esac    

