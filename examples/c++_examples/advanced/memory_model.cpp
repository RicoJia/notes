#include <atomic>
#include <thread>
#include <iostream>
std::atomic<bool> x,y;
void func_1() {
  x.store(true, std::memory_order_relaxed);
  y.store(true, std::memory_order_relaxed);
}
void func_2() {
  while(!y.load(std::memory_order_relaxed));
  if(!x.load(std::memory_order_relaxed)) {
    std::cout << "hehe";  // This might not be executed.
  }
}
int main() {
  for(int i = 0; i< 10000; ++i){

 x = false;
 y = false;
 std::thread thread_1(func_1);
 std::thread thread_2(func_2);
 thread_1.join();
 thread_2.join();
  }
}
