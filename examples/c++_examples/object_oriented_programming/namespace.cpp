/**
 * run this file: 
 * g++ -std=c++17 namespace.cpp
*/
#include <filesystem>
#include <iostream>


// 1. You can assign namespace like this
// Namespace cannot go into a function
namespace foo_namespace = std::filesystem;

// 2. anonymous namespace: this can be seen only by the current cpp file. 
// You can access it as if the namespace doesn't exist.
namespace {
    int i = 1;
}
void test_anonymous_ns(){
    std::cout<<i<<std::endl;
}

int main()
{
    
    test_anonymous_ns();
}
