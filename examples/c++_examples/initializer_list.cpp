#include <initializer_list>
#include <iostream>
#include <vector>

using std::cout; using std::endl; 

void foo(const std::vector<int>& ls){
  std::vector<int> vec; 
  vec.insert(vec.end(), ls.begin(), ls.end()); 
  cout<<ls.size()<<endl;
}

template<typename T>
void bar(T t){}

// most vexing parse
class Foo{
    public:
    Foo(int i){};
    Foo() = default;
}; 

// narrowing is not preferred
class Baz{
    public:
      Baz (std::initializer_list<int> l){}
};

int main()
{
    // narrowing 
    // Baz b({1.0,2,3,4,5,6});

    // automatic conversion
    for (auto i : {1,2,3})  cout<<i<<endl;

    // auto&& universal reference param
    auto foo = [](auto&& func, auto&& params){};

    // most vexing parse
    Foo f(1);   //You're creating an object
    Foo f2();   //May not compile as "function"

}
