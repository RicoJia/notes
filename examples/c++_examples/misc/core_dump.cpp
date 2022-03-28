/**
 * Core dump: 
 *  1. Accessing invalid ptr not initialized 
 *  2. Iterator, container do not have valid element 
 *  3. string using '' instead of ""
 *  4. Threads accesing invalid data
 *  5. Use sigquit (kill -3 PID), sigbart
 * Core dump characteristics: 
 *  1. Is silent, not catchable. Sometimes it will show, sometimes not. 
 *  2. Core means "memory". Dump means thrown out. When exits, core gets dumped into a file called core.进程号
 *  3. gdb -c core. Make sure core file switch is on, environment: ulimit -c unlimited 
 *  4. Sometimes the process may not be down! 
*/

#include <iostream>
#include <map>
using std::cout; using std::endl; 

void test_invalid_ptr(){
    char* n; 
    for (unsigned int i = 0; i < 100; ++i) {
       cout<<n[i]<<endl; 
    }
    cout<<n; 
}

void test_empty_container(){
    for (unsigned int i = 0; i < 100; ++i) {
        std::map<int, int> mp; 
        auto itr = mp.find(20); 
        cout<<itr->second<<endl;
    }
}

void test_single_quotes(){ 
    //sometimes this will be compile error
    // char* c = 'aasdfasd'; 
    // cout<<c;
    while(1){}
}

int main()
{
    test_invalid_ptr();
    test_empty_container();
    test_single_quotes();
}
