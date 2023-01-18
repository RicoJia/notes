#include <iostream> 
// defines std::to_integer
#include <cstddef> 

using std::cout; using std::cin; using std::endl;

void test_bytes(){
    // std::bytes are introduced in C++ 17. Can be initialized with any number {0..255}
    std::byte x{20};
    // need <<= to get the number
    x <<= 1;
    x >>= 2;
    cout<<"x now is: "<<std::to_integer<int>(x)<<endl;
}

int main(int argc, char** argv){
    test_bytes();
}