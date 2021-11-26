#include <map> 
#include <iostream>
using std::cout; using std::endl; 

int main()
{
  std::multimap<int, int> mp; 
  mp.insert({1,2}); 
  mp.insert({0,300}); 
  mp.insert({-1,100}); 
  
  // for (auto i = mp.begin(); i != mp.end(); ++i) {
  //   cout<<i->second<<endl;
  // }
  cout<<mp.rbegin()->second<<endl;
  cout<<std::min(1,2);
}
