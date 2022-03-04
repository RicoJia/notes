#include <iostream>
using std::cout; using std::endl; 

int main()
{
  int arr[] = {1,2,3};  
  auto [x,y,z] = arr; 
  cout<<x<<y<<z;
  
}
