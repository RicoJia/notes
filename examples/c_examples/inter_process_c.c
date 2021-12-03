/**
 * Compilation: 
 *  1. preprocessing: gcc -E main.c:  Produces no file
 *      1. remove comments
 *      2. include header files
 *      3. replaces all macros with value
 *  2. Compilation: generate assembly code
 *      1. gcc -s main.c
 *  3. Assembling: generate machine code with place holders
 *      1. gcc -c main.c, generate .o file
 *  4. Linking: 
 *      1. Link all object files together. 
 *      2. Link function calls with definitions, including theiry libs.
 *          - static lib: copied to the binary
 *          - dynamic lib, by default. Just the name in the binary.
 */


#include <stdio.h>

int main(int argc, char *argv[])
{
    setbuf(stdout, NULL); // print to stdout  without buffering, stdio
    printf("%s", "STM32 is great");
    return 0;
}
