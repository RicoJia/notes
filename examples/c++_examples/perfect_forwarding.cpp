#include <iostream>
#include <utility>
using std::cout; using std::endl; 
#include <type_traits>
#include <vector>

// 1. type deduction
template <typename T>
void f(T&& param){
  foo(std::forward<T> (param));
}

void foo(const std::vector<int>& v){
    cout<<"foo, vec"<<endl;
}
void type_deduction(){
        // Error: does not get type-deducted for templated functions.
        f({1}); 
}

// 2. rvalue ref functions
class Foo{
    public:
        Foo() = default;		//default constructor won't be generated automatically
        Foo(Foo&& ){cout<<"Foo move ctor"<<endl; }		//move constructor for constructing using a r-value
};

class Bar{
    public:
        Bar(){}
        // Does not go well with Bar().getFoo()
        Foo& getFoo()& {cout<<"lvalue foo is returned "<<endl; return foo;}	//foo is already lvalue

        // Will return Foo&, but later will trigger r_foo(const Foo&). So not efficient
        // Also, cannot be overloaded with two getFoo()&& functions
        // Foo& getFoo()&& {
        //    cout<<"rvalue foo is returned "<<endl;
        //    return foo;
        // }

        // good
        //move is key
        Foo&& getFoo()&& {cout<<"rvalue foo is returned "<<endl; return std::move(foo);}
    private:
        Foo foo;
};

void rvalue_ref_func(){
    //r_foo is move-constructed
    auto r_foo = Bar().getFoo();
}

int main()
{
    type_deduction(); 
    // rvalue_ref_func();
}
