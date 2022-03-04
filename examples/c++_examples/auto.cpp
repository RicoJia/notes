#include <iostream>
using std::cout; using std::endl; 

class Auto
{
public:
  Auto (auto i):i_(i){

  }

private:
  const auto i_; 
};

int main()
{
  Auto(1); 
}
