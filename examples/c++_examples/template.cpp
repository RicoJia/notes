#include <iostream>
#include <string>
#include <math.h>
using namespace std;

// template <typename T1, class T2>			// LOL you can use typename and class interchangeably. 
// T2 sum (T1 i, T2 f)
// {
// return (T2)(i+f);
// }
//
// template <class T>
// T product (T a, T b)				// DETAIL: use () instead of <>. 
// {return a*b; 					// single type. Remember to return something!!!
// }
//
//
// template <class T1, class T2>
// class fruits
// {
// T1 apple_num;
// T2 pear_num;
// public:
// fruits(T1 apple, T2 pear):apple_num(apple), pear_num(pear)
// {}
// void access() const 
// {cout<<"Apple is :"<<apple_num;
//  cout<<"Pear is : "<< pear_num;
// }
// }; 		//why not fruta?
//
// int main ()
// {
// cout<<"sum of 5 and 6.5 is"<<sum(5,6.5)<<endl; 
// cout<<"sum of 6.5 and 5 is"<<sum(6.5,5)<<endl; 
//
// cout<<"the product of 5 and 6 is "<< product(5,6)<<endl;
// cout<<"the product of 5.5 and 6.7 is "<< product(5.5,6.7)<<endl;
//
// fruits<float, int> fruta(4.333,5); 		//!! you need to indicate return tyoe here fruits<float, int>!!!
// fruta.access(); 
//
// return 0;
// }
//

// Test 2 - template specialization
// primary template
template<typename T>
void is_voi(T input){};

// explicit specialization for T = char
// template<>
// void is_voi <char> (char input){};

//equivalent to above, since template arguments can be deducted from the function arguments.
template<>
void is_voi(char input){}

