#include <iostream> 
using std::cout; using std::cin; using std::endl;

void test_gnu_pack(){
    struct Foo{
        // long is two bytes. Compiler aligns i and j (so i has two bytes instead of 1)
        // Because on some machines this will be faster.
        int i; 
        long j;
    }; 

    struct [[gnu::packed]] FooPacked{
        int i; 
        long j;
    };

    struct __attribute__((packed)) FooPackedEquivalent{
        int i;
        long j;
    };

    cout<<"size Foo: "<<sizeof(Foo)<<endl;
    cout<<"size FooPacked: "<<sizeof(FooPacked)<<endl;
    cout<<"size FooPackedEquivalent: "<<sizeof(FooPackedEquivalent)<<endl;
}

int main()
{
    test_gnu_pack();
}