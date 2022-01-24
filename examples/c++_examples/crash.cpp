#include <vector>
#include <iostream>

using std::cout; using std::endl; 

int main()
{
    std::vector<int> vec(100,0); 
    for (unsigned int i = 0; i < 101; ++i) {
        vec.at(i) = 100; 
        cout<<i<<endl;
    }
    return 0;
}
