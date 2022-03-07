#include <iostream>
#include <thread>
#include <future>

int addNumbers(int a, int b) {
  return a + b;
}
// anonymous class cannot be passed in as argument, so not working with std::bind
struct {
    void operator()(){
      std::packaged_task<int(int, int)> pt(addNumbers);
      std::future<int> fut = pt.get_future();
      //You have to launch it on a thread, which is THE GREATEST ADVANTAGE OF packaged_task
      std::thread t(std::move(pt), 1, 2);     
      std::cout << "The result is: " << fut.get() << "\n";
      t.join();

      // or use a lambda
      std::packaged_task<int(int, int)> pt2(
        [](int i, int j){return i+j;}
      ); 
      std::future<int> fut2 = pt2.get_future(); 
      std::thread t2(std::move(pt2),1 ,2); 
      std::cout << "The result is: " << fut2.get() << "\n";
      t2.join();
    }
}test_packaged_task_basic;

int main() {
    test_packaged_task_basic();
}
