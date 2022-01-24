#include <iostream>
#include <string>
using std::cout; using std::endl; 

struct CallbackCallerBase{
public: 
  // CallbackCallerBase(){}
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

int main()
{
  CallbackCallerBase* caller = new CallbackCaller<std::string>();
  std::string msg = "123";
  caller->call(&msg);
}
