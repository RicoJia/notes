message("Inside subdirectory: Before modification: ${Name}")
set(Name Bob PARENT_SCOPE)
message("Inside subdirectory: After modification: ${Name}")

