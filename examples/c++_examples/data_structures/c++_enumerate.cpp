#include <vector>
#include <iostream>
#include <type_traits>
using std::cout; using std::endl; 

#include <utility>
#include <tuple>
template <typename T,
          typename TIter = decltype(std::begin(std::declval<T>())),
          typename = decltype(std::end(std::declval<T>()))
          >
constexpr auto enumerate(T&& iterable){
    struct iterator{
        size_t i;
        TIter iter;     // not const_iter

        bool operator != (const iterator& other){return other.iter != iter; }
        void operator ++(){
            ++i; ++iter;
        }

        auto operator *(){
            if (std::is_lvalue_reference<decltype(std::declval<int>())>::value) cout<<"is lvalue"<<endl;
            else cout<<"not lvalue"<<endl;

            return std::tie(i, *iter);
        }

    };

    struct iterable_wrapper{
        T iterable;
        auto begin(){return iterator{0, iterable.begin()}; }
        auto end(){return iterator{0, iterable.end()}; }

    };

    return iterable_wrapper{std::forward<T>(iterable)};
}

// std::begin(rvalue ref) is itr, not const_itr? 
int main()
{

    std::vector<int> vec {1,2,3,4};
    for (const auto&[index, item]: enumerate(vec)){
        item = 100; 
    }
    
    for(auto i: vec){
        cout<<i<<endl;
    }

    // foo(vec);
}
