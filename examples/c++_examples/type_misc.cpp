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

void test_type_def(){
  typedef unsigned int ui;
  // unsigned automatically translate -1 to its positive value.
  ui k = -1; 
  float l = 0.1111;
  auto p = k;
  p = l;
  cout << p << endl;
}
int main() {
    test_type_def();
}
