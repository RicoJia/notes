#include <memory>
#include <iostream>
using std::cout; using std::endl; 

struct Foo{
  Foo(){
    std::cout<<__FUNCTION__<<"1"<<std::endl; 
  }
};

struct Bar{
  int i_;
};

int main()
{
  auto ptr = std::unique_ptr<Bar>(new Bar{2}); 
  cout<<ptr->i_<<endl;
  
}


