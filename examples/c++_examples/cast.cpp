#include <iostream>
#include <cmath>

using std::cout; using std::endl; 

void test_cast(){
   double i = 1.9; 
   // see 1
   cout<<static_cast<int>(i)<<endl;
   cout<<static_cast<unsigned int>(i)<<endl;
   // see 2. cmath is required
   cout<<ceil(i)<<endl; 

   i = -1.9;
   // see -1
   cout<<static_cast<int>(i)<<endl;
   // see 2^32-1
   cout<<static_cast<unsigned int>(i)<<endl;
}

int main()
{
    test_cast();  
}
