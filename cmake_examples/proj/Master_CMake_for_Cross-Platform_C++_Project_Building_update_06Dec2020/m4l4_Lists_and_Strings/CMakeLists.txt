cmake_minimum_required(VERSION 3.0.0)

set( VAR a b c;d "e;f" 2.7 "Hello There" )

list(APPEND VAR 1.6 XX)
message( ${VAR} )

list(REMOVE_AT VAR 2 -3)
message( ${VAR} )

list(REMOVE_ITEM VAR a 2.7)
message( ${VAR} )

list(INSERT VAR 2 XX 2.7)
message( ${VAR} )

list(REVERSE VAR)
message( ${VAR} )

list( REMOVE_DUPLICATES VAR )
message( ${VAR} )

list( SORT VAR)
message( ${VAR} )

list( LENGTH VAR len_var )
list( GET VAR 2 5 6 sub_list )

#Note: 'SUBLIST' and 'JOIN' subcommands were introduced in cmake version 3.12.4. if you have an older version, these commands won't work

#list( SUBLIST VAR 2 3 sub_list2 )
#list( JOIN VAR ++ str_list )
list( FIND VAR XX find_var )

message( "len_var: ${len_var}" )
message( "sub_list: ${sub_list}" )
#message( "sub_list2: ${sub_list2}" )
#message( "str_list: ${str_list}" )
message( "find_var: ${find_var}" )


message("______STRING_____")
set(VAR "CMake for Cross-Platform C++ Projects")

string(FIND ${VAR} "for" find_var)
message(${find_var})

string(FIND ${VAR} "For" find_var)
message(${find_var})


string(REPLACE "Projects" "Project" replaced_var ${VAR} )
message(${replaced_var})

string(PREPEND replaced_var "Master ")
message(${replaced_var})

string(APPEND replaced_var " Building")
message(${replaced_var})

string(TOLOWER ${replaced_var} lower_case_var)
message(${lower_case_var})

string(TOUPPER ${lower_case_var} upper_case_var)
message(${upper_case_var})


string(COMPARE EQUAL ${upper_case_var} "MASTER CMAKE FOR CROSS-PLATFORM C++ PROJECT BUILDING"  equality_check_var)
message(${equality_check_var})

string(COMPARE GREATER ${upper_case_var} "some random string"  greater_check_var)
message(${greater_check_var})









