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

void func(const dumplings& d){
    d.baz(); 
}

int main()
{
  dumplings corn(4);
  func(corn); 
  // corn.value = 60;
  // int i = corn.foo();
  // cout << i << endl;

  // int j = 29;
  // int *const p = &i; // BE VERY CAUTIOUS HERE ABOUT THE SPELLING: int const *p =
  //                    // &i; is invalid!!
  // const int *q = &j;
  // j = j + 1;
  // cout << "j is: " << j << "*q is" << *q << endl;

  return 0;
}

