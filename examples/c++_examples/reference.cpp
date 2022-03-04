#include <iostream>
#include <utility>
using std::cout; using std::endl; 
#include <type_traits>

// 1. prvalue 
struct Baz{};
Baz baz(){
    return Baz();
}

void prvalue_ctor(){
    Baz f;
    baz() = f; // use default Baz& Baz(const Baz&), which returns an xvalue in this case. Treat it as a valid object until termination.
}

// 2. const r value ref
template <typename T>
void const_r_value_ref(const T&& param){
    // const T&& param is actually const r-value reference
    // if param == 1, T is int, decltype(param) is int&&
    cout<<std::is_rvalue_reference<T>::value<<endl;
    cout<<std::is_rvalue_reference<decltype(param)>::value<<endl;
}

// 3. perfect fowarding 
template<typename T>
void regular_r_ref(T&& param){
    cout<<std::is_rvalue_reference<decltype(param)>::value<<endl;
}

int main()
{
    // const_r_value_ref(1);
    // this is const r value reference
    // Below won't compile: 
    // int i = 1; 
    // const_r_value_ref(i); 
    // So one use is to disallow xvalue and prvalue: template <class T> void ref (const T&&) = delete;
    

    // // perfect forwarding 
    // int&& j = 8; 
    // // regular_r_ref(std::forward<int>(j));
    // cout<<std::is_rvalue_reference<decltype(j)>::value<<endl;   // is rvalue ref
    // regular_r_ref(std::forward<int>(j)); // is rvalue ref
    // regular_r_ref(j); // NOT rvalue ref

}
