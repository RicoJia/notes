#include <iostream>

#include "my_lib/addition.h"
#include "my_lib/division.h"
#include "my_lib/print_result.h"

int main(){

#ifndef RICO_OPTION
// #else
float first_no, second_no, result_add, result_div;

std::cout<< "Enter first number\t";
std::cin>> first_no;
std::cout<< "Enter second number\t";
std::cin>> second_no;

result_add = addition(first_no , second_no);
result_div = division(first_no , second_no);

print_result("Addition", result_add);
print_result("Division", result_div);
//std::cout<< "Addition result:\t"<< result_add<< "\nDivision result:\t"<< result_div<< "\n";
#endif
return 0;

}
