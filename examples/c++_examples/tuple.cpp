#include <tuple>

struct Foo{}; 
Foo foo(){
    return Foo(); 
}

int main()
{
   Foo f; 
   foo() = f; 
}
