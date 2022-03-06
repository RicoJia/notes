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

void test_future_exceptions(){
    // 1. set exception as we wish
    auto promise_set_exception = [](){
        std::promise<int> p; 
        std::future<int> fut = p.get_future();
        std::thread t1(
            [&](){
                p.set_exception(std::make_exception_ptr(std::runtime_error("mine")));
            }
                // but a canonical use of std::promise::set_exception is 
                // try{
                //   throw std::runtime_error("hehe");
                // }
                // catch(...)    //... in C++ will tell compiler NOT to check arg number and types
                // {
                //   // exception_ptr current_exception() noexcept;
                //   p.set_exception(std::current_exception());    // returns the current exception_ptr
                // }
                );
        try{
            std::cout<<__FUNCTION__<<fut.get()<<std::endl;
        }
        catch(...){
            std::cout<<__FUNCTION__<<": exception comes"<<std::endl;
        }
        t1.join(); 
    }; 
    promise_set_exception();

    // 2. broken_promise, because promise is destroyed before setting value
    auto broken_promise_exception = [](){
        std::future<int> fut;
        {
            std::promise<int> p;
            fut = p.get_future();
        }
        fut.get();
    }; 
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
        // future::wait(): blocks until result becomes available, then get() will have no wait
        // - ```valid() == true``` (shared_state)
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
    // test_future_and_shared_future();
    test_future_exceptions();
}
