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
 *  5. to run this program: 
 *      gcc inter_process_c.c && ./a.out
 */


#include <stdio.h>
#include <unistd.h>
#include <signal.h>

void h1(int st){
    printf("%s", ____PRETTY_FUNCTION__); 
}
void h2(int st){
    printf("%s", __PRETTY_FUNCTION__);
}

int main(int argc, char *argv[])
{
    signal(SIGUSR1, h1); 
    signal(SIGUSR2, h2); 
    setbuf(stdout, NULL); // print to stdout  without buffering, stdio
    printf("%s", "STM32 is great");
    // wakes up when you receive a signal 
    pause(); 
    return 0;
}
