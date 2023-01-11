/**
- gcc address_sanitizer.c -fsanitize=address -static-libasan -g
    -g means print line numbers
- Addresssantinizer can detect 
    - memory leaks: LeakSanitizer: detected memory leaks
    - "use after free" (see freed by thread ... )
    - Heap overflow: heap-buffer-overflow 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void test_no_leak(){
    char *s = malloc(100);
    strcpy(s, "Hello world!");
    printf("string is: %s\n", s);
    free(s);

}

void test_missing_free(){
    char *s = malloc(100);
    strcpy(s, "Hello world!");
    printf("string is: %s\n", s);
}

void test_use_after_free(){
    char *s = malloc(100);
    free(s);
    strcpy(s, "Hello world!");
    printf("string is: %s\n", s);
}

void test_overflow(){
    // whoops, forgot c strings are null-terminated
    // and not enough memory was allocated for the copy
    char *s = malloc(12);
    strcpy(s, "Hello world!");
    printf("string is: %s\n", s);
    free(s);
}

int main(int argc, const char *argv[]) {
    // test_missing_free();
    // test_use_after_free();
    test_overflow();
    return 0;
}