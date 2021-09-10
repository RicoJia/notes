#include <thread>
#include <iostream>
#include <future>
#include <chrono>
using namespace std::chrono_literals; 
using namespace std;
 std::promise<void> p;
 void Foo(){
     cout<<"llllee";
    // std::this_thread::sleep_for(10s);
    // p.set_value(); 
 }

 void Bar(){
         cout<<"llllee";

    // p.get_future();    //?
 }

  int main(){
    p(Foo); 
  }

