#include <iostream>
using std::cout; using std::endl; 

class Foo
{
public:
    Foo (): baz_(32){cout<<__FUNCTION__<<endl;}
    ~Foo (){}
    int baz_;
};

// So global variables are properly initialized
Foo f;

int main()
{
    std::cout<<__FUNCTION__<<": baz: "<<f.baz_<<std::endl;

    
}

