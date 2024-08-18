// to compile this file: do g++ foo_main.cpp foo_funcs.cpp. Since all included files are here, we don't need '-I'

#include "foo.hpp"
int num = 100;

int main(int argc, char** argv){
    num_inc();
    std::cout<<"main, num: "<<num<<std::endl;
}