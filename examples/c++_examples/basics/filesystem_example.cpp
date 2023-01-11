#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

void test_file(){
    std::cout<<fs::is_directory("/home")<<std::endl;
}

int main(){
    test_file();
}