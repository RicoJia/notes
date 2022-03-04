#include <iostream>

using namespace std;

class dumplings {
public:
  int value = 99; // C++ forbids in-class initialization of value.
  dumplings(int a) : value(a) {}
  int foo() const // static functions can only access static data members.
  {
    return value;
  }

  int baz(){
    return 1; 
  }
};

#include <memory>
#include <functional>
using Fn = std::function<bool()>; 
class Foo
{
public:
  Foo () : fn_(nullptr){}
  void bar(){
    if(fn_) 
      std::cout<<__FUNCTION__<<"not nullptr"<<std::endl; 
    else
      std::cout<<__FUNCTION__<<"nullptr"<<std::endl; 
  }
private:
  const Fn& fn_ = nullptr;    //dangling reference 
};

// class Baz
// {
// public:
//   Baz ()
//   {
//     f_ = std::make_unique<Foo>(std::bind(&Baz::fn, this));
//   }
//
//   void verify(){
//     f_->bar(); 
//   }
//
// private:
//   std::unique_ptr<Foo> f_; 
//   bool fn(){
//       std::cout<<__FUNCTION__<<"1"<<std::endl; 
//       return false;
//   }
// };

int main()
{
  Foo f; 
  f.bar();
  // const dumplings corn(4);
  // corn.value = 60;
  // int i = corn.foo();
  // cout << i << endl;
  //
  // int j = 29;
  // int *const p = &i; // BE VERY CAUTIOUS HERE ABOUT THE SPELLING: int const *p =
  //                    // &i; is invalid!!
  // const int *q = &j;
  // j = j + 1;
  // cout << "j is: " << j << "*q is" << *q << endl;

  return 0;
}

