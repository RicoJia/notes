#include<iostream>
#include"my_math/addition.h"
#include"my_math/division.h"

int main(){

float first_no, second_no, result_add, result_div;
std::cout<<"Enter 2 numbers\n";
std::cin>>first_no>>second_no;
result_add = addition(first_no, second_no);
result_div = division(first_no, second_no);

std::cout<<"Addition result:\t"<<result_add<<"\n"<<"Division result:\t"<<result_div<<"\n";


return 0;
}
