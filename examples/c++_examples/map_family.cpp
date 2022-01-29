#include <map> 
#include <unordered_map>
#include <iostream>
using std::cout; using std::endl; 

void test_unordered_map(){
    std::unordered_map<int, int> mp = {{3,4}}; 
    // test insertng the same element
    mp[3] = 5; 
    cout<<mp[3]<<endl;
    // using insert, if key already exists, then it wouldn't be updated
    mp.insert({3,6}); 
    cout<<mp[3]<<endl;
    mp.erase(3);
}

void test_map(){
    std::map<int, int> mp = {{3,4}}; 
    // using insert, if key already exists, then it wouldn't be updated
    mp.insert({3,6}); 
    cout<<mp[3]<<endl;
    mp.erase(3);
}

//TODO
void test_multimap(){
  std::multimap<int, int> mmp = {{3,4}};
  // test inserting and iterator
  auto it = mmp.insert({1,2});
  // cout<<it->first<<", "<<it->second<<endl;
  // test insertng the same element
  mmp.insert({1,3});
  
  // no [] or at() in multimap
  // mmp[1] = 4;
  // mmp.at(3) = 4; 
  auto itr_pair = mmp.equal_range(1); 
  for (auto i = itr_pair.first; i != itr_pair.second; ++i) {
    cout<<i->first<<" "<<i->second<<endl;
  }

  // for (auto i = mmp.begin(); i != mmp.end(); ++i) {
  //   cout<<i->second<<endl;
  // }
}

int main()
{
    // test_unordered_map();
    // test_map(); 
    test_multimap();
}
