
#include <iostream>
using std::endl;
using std::cout;

/*
 * V table.
 *  - C++ doesn't require how polymorphism is implemented. But de facto, every class has a vtable. It's usually the first element of the generated structure.
 *      - It looks like: 
 *          &foo -> derived foo
 *  - base class ptr contains a vpointer to the vtable, listing all virtual functions
 *  - at contruction, child class copies the parent's vtable. If you have virtual function, the vtable will contain it.
 *  - During compile time, code is generated to find the location of vtable
 *  - vptr is looked up during run time. offset of the function (each function has a fixed offset)
 */
void test_polymorphism(){
    class Parent{public: void foo(){cout<<" Parent foo"<<endl;}};
    class Child: public Parent{public: void foo(){cout<<" Child foo"<<endl;}};
    Child* bptr = new Child();
    bptr->foo();    // calling the child's foo
    delete bptr;
}

int main(){
    test_polymorphism();
}
