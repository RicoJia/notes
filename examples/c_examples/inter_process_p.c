#include <stdio.h>
#include <signal.h>

int main(int argc, char *argv[])
{
    pid_t pid = atoi (argv[1]); 
    kill(pid, SIGUSR1); 
    kill(pid, SIGUSR2); 
    printf("Sent usr1 to %d: ", pid);
    return 0;
}
