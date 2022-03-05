#include <vector>
#include <iostream>
#include <memory>

using std::cout; using std::endl; 

void make_unique_crash()
{
   struct crash_t {
       int i; 
       int j; 
   };     

   // 1. Never use {} because make_unique is a function
   // auto ptr = std::make_unique<crash_t>{1,2}; 
   // 2. make_unique, make_shared does NOT work with list intialization
   // auto ptr = std::make_unique<crash_t>(1,2);
   // auto shared_ptr = std::make_shared<crash_t>(1,2); 
   // 3. list initializer should use {}, not ()
   // auto ptr = std::unique_ptr<crash_t>(new crash_t(1,2)); 
   auto ptr = std::unique_ptr<crash_t>(new crash_t{1,2}); 
}

#include <mutex>
void mtx_ptr_in_struct()
{
    struct crash_t {
        std::mutex* mtx_; 
    };    
}

int main()
{
    // make_unique_crash();
    mtx_ptr_in_struct();
}
