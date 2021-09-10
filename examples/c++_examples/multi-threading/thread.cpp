#include <iostream>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using namespace std; 

void func(){
    for (int i = 0; i < 10; ++i){
        cout<<"func"<<endl;
        // std::this_thread::sleep_for(10ms); 
        
    }
}

std::thread ret_th(){
    std::thread t3(func); 
    return t3; 
}

int main()
{
    std::thread t1(func);
    // std::this_thread::sleep_for(2s); 
    std::thread t2 = std::move(t1);
    // auto t3 = ret_th();
    // t2.join(); 
    return 0;
}

