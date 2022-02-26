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
      // try accessing the param
      cout<<i_<<endl;
  }
private:
  /* data */
};

int main()
{
    Bar bar;  
    bar.func();
    // test out protected data
    cout<<bar.i_<<endl;
}
