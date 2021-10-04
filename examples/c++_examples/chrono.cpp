#include <chrono>
#include <iostream>
using std::cout; using std::endl; 
using namespace std::chrono; 
using namespace std::chrono_literals;

int main()
{
  duration<double, std::ratio<1,1>> dur = system_clock::now().time_since_epoch(); 
  // cout<<dur.count() * system_clock::period::num / system_clock::period::den<<endl;
  cout<<dur.count()<<endl;
}
