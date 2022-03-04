#include <iostream>
using std::cout; using std::endl; 

class Foo{
    public: 
        // This is declaration, not definition
        static int i; 
        static int j_ = 3; 
        // can declare static func inside class
        static void foo_static(){
            cout<<__FUNCTION__<<endl;
        }
        // calls static, if in the same class, no need to add class name
        void foo(){
            foo_static();
            Foo::foo_static();
        }
};

class Child:public Foo{
    public:
        void foo(){
            // even child class can call the base class static func
            foo_static();
        }
}; 

// This is how to initialize static member func, this is definition
// If in hpp, then every instance will get the same value. Else, should be in the source file.
int Foo::i(1); 

int main()
{
    Foo f;
    f.foo(); 
    cout<<Foo::i<<endl; 
    Foo::foo_static();

    Child c; 
    c.foo();
    return 0;
}
