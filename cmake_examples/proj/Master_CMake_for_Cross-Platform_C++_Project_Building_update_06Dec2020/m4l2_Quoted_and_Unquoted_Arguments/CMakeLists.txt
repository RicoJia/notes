cmake_minimum_required(VERSION 3.0.0)

set(VAR aa bb cc)
#set(VAR aa;bb;cc)
#set(VAR "aa" "bb" "cc")
#set(VAR "aa bb cc")
#set(VAR "aa;bb;cc")


message(Unquoted: ${VAR})

message(Quoted: "${VAR}")
