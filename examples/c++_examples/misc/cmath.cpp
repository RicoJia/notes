#include <iostream>
#include <cmath>

using std::cout; using std::endl; 

void test_cos()
{
   const double pi = std::acos(-1);
   cout<<"pi/2: "<< pi / 2 <<endl;
}


void test_rounding(){
    cout<<"ceil: next larger int: "<<std::ceil(4.5)<<endl; 
    cout<<"floor: next smaller int: "<<std::floor(4.5)<<endl; 
}

int main()
{
    // test_cos(); 
    test_rounding();
}
