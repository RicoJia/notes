#include "rico.hpp"

int main(void)
{
  int N = 1<<20; // 1M elements
  float x = 1.0f; 
  float y = 2.0f; 
  add(N, x, y); 

  return 0;
}
