#include <iostream>

void test_while_loop(){
    int i = 2;
    // -=, = returns the number as a lvalue reference
    while((i -= 1) != 0){
        std::cout<<i<<std::endl;
    }

}

int main()
{
    test_while_loop();
    
}
