#include <iostream>
using std::cout; using std::endl; 

template <typename T>
class Foo
{
public:
  Foo (T t): i_(t){}

protected:
  /* data */
  T i_; 
};

class Bar: public Foo<int>
{
public:
  Bar (): Foo(1){}
  void func(){
      // try accessing protected param
      cout<<i_<<endl;
  }
private:
  /* data */
};

void test_inheritance(){
    Bar bar;  
    bar.func();
    // test out protected data
    // cout<<bar.i_<<endl;
}
// ==============================================
// template class inheriting from a regular class
#include <iostream>
#include <string>
using std::cout; using std::endl; 

struct CallbackCallerBase{
public: 
  virtual void call(void* msg) =0; 
};

template <typename T>
struct CallbackCaller : public CallbackCallerBase{
  public: 
      CallbackCaller(){}

    void call(void* msg){
      cout<<*(static_cast<T*>(msg))<<endl;  
    }
};

void test_template_inheritance(){
    CallbackCallerBase* caller = new CallbackCaller<std::string>();
    std::string msg = "123";
    caller->call(&msg);
}
int main()
{
    test_template_inheritance();
}
