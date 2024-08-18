// to compile this file: do g++ foo_main.cpp foo_funcs.cpp. Since all included files are here, we don't need '-I'

#include "foo.hpp"
using namespace std;

int num = 100;

// void test_bar(){
//     //TODO
//     std::cout<<"test bar: main"<<std::endl;
// }

class Foo{
    public: 
        // This is declaration, see below for definition
        static int i; 
        // can declare static func inside class
        static void foo_static(){
            cout<<__FUNCTION__<<endl;
        }
        // calls static, if in the same class, no need to add class name
        void foo(){
            foo_static();
            Foo::foo_static();
        }

        void print_i(){
            std::cout<<"i: "<<i<<std::endl;
            ++i;
        }
};

// This is how to initialize static member func, this is definition
// If in hpp, then every instance will get the same value. Else, should be in the source file.
int Foo::i(123); 

int main(int argc, char** argv){
    num_inc();
    num_dec_inside_function();
    num_dec_inside_function();
    std::cout<<"main, num: "<<num<<std::endl;

    Foo f;
    f.print_i();
    std::cout<<"class variable: "<<Foo::i<<std::endl;

    Foo::foo_static();
    // test_bar();
}
