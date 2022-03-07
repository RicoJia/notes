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
    // **Each promise has one associated future**
    std::promise<int> sumPromise;
    //(set up the setter-getter data channel)
    std::future<int> sumFuture = sumPromise.get_future(); 
    std::thread t1 (set_promise, std::move(sumPromise)); 
    std::cout<<__FUNCTION__<<": sum promise: "<<sumFuture.get()<<std::endl;
    t1.join();
    
    /* 2. shared future: you can access it on multiple threads. It's copyable. std::future::valid() == false 
     * 3. [!] multiple threads can access the same promisem but you must have multiple shared_future. one single std::shared_future accessed by different threads will have data race 
     * 4. You can get only one std::future/std::shared_future from std::promise
     */
    std::promise<int> sumPromise2;
    std::shared_future<int> sf3 = sumPromise2.get_future();    //imlicit conversion from std::future
    // std::shared_future<int> sf4 = sumPromise2.get_future();    // Illegal, can't retrieve from future twice
    // std::shared_future<int> sf1 = sumFuture.share(); // Legal, if you already have future
    std::shared_future<int> sf4 = sf3; // Legal
    auto func = [&](std::shared_future<int> sf){
        // std::future::wait(): blocks until result becomes available, then get() will have no wait
        // std::future::valid() == true (shared_state)
        sf.wait(); 
        std::cout<<__FUNCTION__<<": sf: "<<sf.get()<<std::endl;
    }; 
    auto result_fut_1 = std::async (std::launch::async, func, sf3);
    auto result_fut_2 = std::async (std::launch::async, func, sf4);
    // This will never cause hanging, like conditional_variable notify_one
    sumPromise2.set_value(99); 
    result_fut_1.get(); 
    result_fut_2.get(); 
}

/**
 * Theory 
 * 1. if args are passed in as rvalues, the copies are created by **moving**, just like std::thread. 
 * 2. Can be implemented on top of std::packaged_task
*/
void test_async_construct(){
    // std::future std::async (T...) // it's a template function
    // 1. contstruction
    class Foo
    {
    public:
        Foo () = default;
        Foo(Foo&&){ std::cout<<__FUNCTION__<<": move ctor"<<std::endl; }
        Foo(const Foo&){ std::cout<<__FUNCTION__<<": copy ctor"<<std::endl; }
        void bar(){
            std::cout<<__FUNCTION__<<": bar ================"<<std::endl;
        } 
    };
    Foo x; 
    auto f1 = std::async(&Foo::bar, x);   // creates a copy of x inside async, then move because of internal implementation
    f1.get();
    auto f2 = std::async(&Foo::bar, &x);    // pointer, no copy/move
    f2.get();
    auto f3 = std::async(&Foo::bar, std::ref(x));     //reference, no copy, move. Not you have to use std::ref!
    f3.get();
    auto f4 = std::async(&Foo::bar, Foo());     // after default ctor, then move ctor is used, then another move because of internal implementation
    f4.get();
}

/**
 * Theory: 
 * 1. std::async by default will randomly start running the function immediately / defer its start, on a separate thread. But you have control over when this will be run
 * 2. launch::deferred may never get executed, or in this case, depends on the scheduling of the thread it runs on.
 * 3. ```std::launch::deferred``` and ```std::launch::async``` are in the enum ```launch```
 * 4. You can check if the ```std::future_status::deferred```
*/
void test_async_launch(){
    auto asyncLazy=std::async(std::launch::deferred,[]{ return  std::chrono::system_clock::now();});
    auto asyncEager=std::async( std::launch::async,[]{ return  std::chrono::system_clock::now();});
    using namespace std::chrono_literals;
    switch (asyncLazy.wait_for(std::chrono::seconds(0))) {
        case std::future_status::timeout:
        std::cout << "thread still running\n";
        break;
        case std::future_status::ready:
        std::cout << "thread done, outcome available\n";
        break;
        case std::future_status::deferred:
        std::cout << "thread was deferred\n";
        break;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout<<__FUNCTION__<<": eager start: "<<(asyncEager.get().time_since_epoch().count())/1000000000<<std::endl;
    std::cout<<__FUNCTION__<<": lazy start: "<<(asyncLazy.get().time_since_epoch().count())/1000000000<<std::endl;
}


/**
* Theory: 
* 1. ```std::thread``` cannot handle exceptions from the function. you will get ```std::terminate``` 
* 2. If you create too many ```std::threads```, you'll get ```std::system_error```, even tho the function is **noexcept**
*/
void test_async_exception(){
    auto fut = std::async([](){throw "lol";}); 
    try{
        fut.get(); 
    }
    catch (char const* str){
        std::cout<<__FUNCTION__<<": exception: "<<str<<std::endl;
    }
}


int main() {
    // test_packaged_task_basic();
    // test_future_and_shared_future();
    // test_future_exceptions();

    // test_async_construct();
    test_async_launch();
    // test_async_exception();
}
