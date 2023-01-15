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
            // std::declval returs an "added rvalue reference": https://www.cplusplus.com/reference/type_traits/add_rvalue_reference/
            // Also, there's reference collapsing 
            // here, if we pass in vector, then it's passed in as lvalue ref &
            // that's why std::declval<T>() returns lvalue ref
            if (std::is_lvalue_reference<decltype(std::declval<T>())>::value) 
                cout<<"is lvalue ref"<<endl;
            else cout<<"not lvalue ref"<<endl;

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
        // item = 100; 
    }
}
