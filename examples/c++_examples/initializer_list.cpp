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

int main()
{
  // for (auto i : {1,2,3})  cout<<i<<endl;
  // foo({1,2,3,4,5,6});

    // auto&& universal reference param
    auto foo = [](auto&& func, auto&& params){};
}
