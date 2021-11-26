#include <iostream>
#include <string.h>
using std::cout; using std::endl; 
int main()
{
  int arr[] = {1,2,3};
  int arr2[3]; 
  memcpy(arr2, arr, 3*sizeof(int)); 
  auto [x,y,z] = arr2; 
  cout<<x<<y<<z;
}
