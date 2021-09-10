#include <memory>
#include <iostream>

struct Foo{
  Foo(){
    std::cout<<__FUNCTION__<<"1"<<std::endl; 
  }
};

int main()
{
  std::unique_ptr<Foo> i; 
  {
  }
  
}


