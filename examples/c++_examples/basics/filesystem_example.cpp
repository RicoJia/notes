#include <filesystem>
#include <iostream>
#include <string>
#include <cstdio>

namespace fs = std::filesystem;
using namespace std;

void test_file(){
    std::cout<<fs::is_directory("/home")<<std::endl;
}

std::string make_temp_directory(){
    char temp_name[] = "/tmp/rico_test.XXXXXX";  // NOLINT: clang-tidy doesn't like C arrays
    // mkdtemp shall ensure that the string provided in template is a pathname ending with at least six trailing 'X' characters
    const auto* temp_directory = mkdtemp(static_cast<char*>(temp_name));
    printf("%s", temp_directory);
    // return std::string{ temp_directory };
    return "";
}

int main(){
    // test_file();
    cout<<make_temp_directory()<<endl;
}