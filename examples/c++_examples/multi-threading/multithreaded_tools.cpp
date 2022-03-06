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
      //You have to launch it on a thread
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

void set_promise(std::promise<int> p){
    p.set_value(12); 
}

void test_future_and_shared_future(){
    // 1. cannonical future
    std::promise<int> sumPromise;
    //(set up the setter-getter data channel)
    std::future<int> sumFuture = sumPromise.get_future(); 
    std::thread t1 (set_promise, std::move(sumPromise)); 
    std::cout<<__FUNCTION__<<": sum promise: "<<sumFuture.get()<<std::endl;
    t1.join();
    
    // 2. shared future: you can access it on multiple threads. It's copyable
    // You can get only one std::future/std::shared_future from std::promise
    std::promise<int> sumPromise2;
    std::shared_future<int> sf3 = sumPromise2.get_future();    //imlicit conversion from std::future
    // std::shared_future<int> sf4 = sumPromise2.get_future();    // Illegal, can't retrieve from future twice
    // std::shared_future<int> sf1 = sumFuture.share(); // Legal, if you already have future
    // std::shared_future<int> sf4 = sf3; // Legal

    // 3. you can push function accessing the same std::shared_future
    auto func = [&](){
        sf3.wait(); 
        std::cout<<__FUNCTION__<<": sf: "<<sf3.get()<<std::endl;
    }; 
    auto result_fut_1 = std::async (std::launch::async, func);
    auto result_fut_2 = std::async (std::launch::async, func);
    // This will never cause hanging, like conditional_variable notify_one
    sumPromise2.set_value(99); 
    result_fut_1.get(); 
    result_fut_2.get(); 
}

void test_async(){
    // std::future std::async (T...) // template function
    // launch::deferred may never get executed 
    // thread local storage, 
}

int main() {
    // test_packaged_task_basic();
    test_future_and_shared_future();
}
