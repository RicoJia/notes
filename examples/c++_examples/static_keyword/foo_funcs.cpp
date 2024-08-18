#include "foo.hpp"

static int num = 0; 

void num_inc(){
    num++;
    std::cout<<"num: "<<num<<std::endl;
}