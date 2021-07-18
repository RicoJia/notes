cmake_minimum_required(VERSION 3.0.0)

set(Name Charlie)

function(print_detail)
	message("Inside function: Before modification: ${Name}")
	set(Name Bob PARENT_SCOPE)
	message("Inside function: After modification: ${Name}")
endfunction()

message("Outside function: before function call: ${Name}")
print_detail()
message("Outside function: after function call: ${Name}")
