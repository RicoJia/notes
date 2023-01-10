/**
 * Run this file: gcc basics.c -o out && ./out
 */

#include <stdio.h>

void test_print(){
    // need <stdbool.h> because it's not a standard type in c
    #include <stdbool.h>
    printf("%d\n", !!false);

}

void test_int_type(){
    printf("%lu %s", 1UL<<10, "this is unsigned long");
}

int main(){
    test_int_type();
}
