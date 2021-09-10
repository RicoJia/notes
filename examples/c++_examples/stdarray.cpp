#include <iostream>
#include <array>
// use c++ 11 compiler to compile this if you have error compiling <array>		g++ -std=c++11 -o filename filename.cpp 

using namespace std;

void changearray(std::array <int,5> & n) ;

int main(){

std::array <int,5> a ={1,2,3}; 					
std::cout << a[2]<< std::endl; 			// Accessing a std:; array. you will get 3. 
std:: cout<< a.size()<<std::endl;			// this gives you the max length of the array.

changearray(a);
std::cout << a[2] << std::endl;

return 0;  
}

void changearray(std::array<int, 5> &n) { n[2] = 100; }
