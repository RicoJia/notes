#include <boost/lockfree/spsc_queue.hpp>
#include <thread>
#include <iostream> 
using std::cout; using std::cin; using std::endl;

void test_single_consumer_single_producer_queue(){
    // This class is an optimized circular buffer for use cases 
    // Where exactly one thread writes to the queue and exactly one thread reads from the queue. 
    // See https://theboostcpplibraries.com/boost.lockfree
    boost::lockfree::spsc_queue<int> q(1024);
    constexpr int EXIT = -1;
    auto th = std::thread(
        [&q, EXIT](){
            while (1){
                int num;
                q.pop(&num, 1);
                if ( num == EXIT){
                    cout<<"exiting"<<endl;
                    return;
                }
                else{
                    cout<<"received: "<<num<<endl;
                }
            }
        }
    );
    q.push(1);
    q.push(EXIT);
    th.join();
}

int main(){
    test_single_consumer_single_producer_queue();
}