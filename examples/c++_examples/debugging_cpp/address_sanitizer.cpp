/**
 - g++ address_sanitizer.cpp -fsanitize=address -g
 */
#include <cstring>
#include <iostream>

// see alloc-dealloc-mismatch
void test_delete_mismatch(){
    char *cstr = new char[100];
    strcpy(cstr, "Hello World");
    std::cout << cstr << std::endl;
    delete cstr;
}

int main(){
    test_delete_mismatch();

}
