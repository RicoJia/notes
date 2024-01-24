#include <bits/types/clock_t.h>
#include <vector>
#include <iostream>
#include <string>
#include <memory>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

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

   // default intiailization itself doesn't cause crash
   auto default_obj = crash_t(); 
   default_obj.i = 9;
   // But this can cause crash, because this is not default ctor, it's just nullptr
   auto default_ptr = std::unique_ptr<crash_t>(); 
   // This is the real default ctor using unique_ptr
   default_ptr = std::unique_ptr<crash_t>(new crash_t()); 
   if (default_ptr) std::cout<<__FUNCTION__<<": valid default ptr"<<std::endl;
   default_ptr->i = 9;
}

#include <mutex>
void mtx_ptr_in_struct()
{
    struct crash_t {
        std::mutex* mtx_; 
    };    

    struct Foo {
        public: 
            Foo(std::unique_ptr<crash_t>&& ptr){
                if (ptr->mtx_ != nullptr) std::cout<<__FUNCTION__<<": mtx ptr valid"<<std::endl;
                std::unique_lock<std::mutex> ul(*(ptr->mtx_)); 
                std::cout<<__FUNCTION__<<": victory"<<std::endl;
            }
        private: 
            std::unique_ptr<crash_t>ptr_; 
    };
    
    class Bar
    {
    public:
        Bar (): f_(std::unique_ptr<crash_t>(new crash_t{&mtx_}))
        {

        }
    
    private:
        std::mutex mtx_;
        Foo f_;
    };

    // This is fine actually. just remember to move(ptr), if ptr is already an object
    Bar();
}

/**
 * Notes:
 * 1. constexpr only works with literal types (POD), because they are guaranteed
 *    to be known. std::string has dynamic memory allocation, so it's not.
 *    So, constexpr std::string_view (cpp 17+) or constexpr char* are allowed
 * 2. std::perror(const char*). std::string -> const char* doesn't happen naturally
 * 
 */
void test_perror(){
    const std::string file_name = "non_existent_file";
    int fd = open(file_name.c_str(), O_RDONLY);

    // this wouldn't be seen as a system error,
    // so error number will be 0. perror will see it as success 
    try{
        throw std::runtime_error("test_perror");
    } catch(...){
        std::perror("test_perror");
    }

    if (fd == -1){
        std::perror((file_name + "Error Opening File ").c_str());
        std::cerr << "Error code: " << errno << std::endl;
    }
    else{
        close(fd);
    }
    
}

#include <exception>
/**
 * @brief : std::exception is the parent class of all cpp exceptions
 * See all exceptions: https://rollbar.com/guides/cpp/how-to-handle-exceptions-in-cpp/#
 */
void test_invalid_argument(){
    try{
        throw std::invalid_argument("test_invalid_argument");
    }
    catch (const std::exception& e){
        std::cout<<e.what()<<std::endl;
    }
}

int main()
{
    // make_unique_crash();
    // mtx_ptr_in_struct();
    // test_perror();
    test_invalid_argument();
}
