#include <iostream>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using namespace std; 

// there's no nested functions in cpp other than lambda. 
// But there's struct and operator (), and we can put a structin a function
void test_thread_move(){
    struct {
        void operator()(){
            for (int i = 0; i < 10; ++i){
                cout<<"func"<<endl;
                std::this_thread::sleep_for(0.6s); 
            }            
        }
    }func;

    std::thread t1(func);
    std::this_thread::sleep_for(0.5s); 
    // the program will keep running! 
    std::thread t2 = std::move(t1);
    std::cout<<__FUNCTION__<<": moved"<<std::endl; 
    t2.join(); 
    // don't need this since t1 is moved from
    // t1.join();
}


int main()
{
    test_thread_move();
    return 0;
}

