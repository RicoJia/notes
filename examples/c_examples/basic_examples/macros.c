#include <stdio.h>
#include <stdbool.h>

// !! will convert a number to bool
#define CHECK(expr)                                                                                           \
  do {                                                                                                                 \
    if (!__builtin_expect(!!(expr), 0)) {                                                                              \
      printf("%s | %d | %s", __FILE__, __LINE__, (#expr));                                                \
    }                                                                                                                  \
  } while (false);

int main(int argc, char *argv[])
{
    CHECK(0);    
    return 0;
}
