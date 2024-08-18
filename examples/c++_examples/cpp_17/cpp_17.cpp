/**
 * compile this file:  
 */

#include <optional>
#include <iostream>
#include <vector>
#include <numeric>
using namespace std;

void test_optional(){
    auto get_ll = [](bool does_return)-> std::optional<size_t> {
        if(does_return) 
            return 1;
        else 
            return {};
    };
    cout<<get_ll(true).value()<<endl;
    if (get_ll(false).has_value()){
        cout<<"yay"<<endl;
    }
}

void test_reduce(){
    std::vector<int> vec {1,2,3,4,5};
    // FUCK: need to have initial value = 0.0, otherwise, it will be int, which round (proper rounding)!! 
    double sum = std::reduce(vec.begin(), vec.end(), 0.0, [](int a, double sum){return 0.25*a + sum;});
    cout<<"using reduce to find sum: "<<sum<<endl;
}

int main(){
    // test_optional();
    test_reduce();

}