#include "rico.hpp"
#include "histogram_counter.cuh"

int main(void)
{
  int N = 1<<20; // 1M elements
  float x = 1.0f; 
  float y = 2.0f; 
  add(N, x, y); 
  Foo();

  return 0;
}
