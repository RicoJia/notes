/**
 * if you were there, what would you wish to know?
what would you wish to do?
    - first is to know how exactly the algorithm works. Coming to realize we need splicing was a big change.
    - naming could be more descriptive
    - coming back to the basic concepts: "does the iterator point to a different element after std::partition, or list::splice?"
    - debug with the simplest case: it shouldn't be any simpler,e.g, array with two elements.
 */

#include <climits>
#include <random>
#include <cassert>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <future>
#include <list>

using std::cout; using std::endl; 
using It = std::list<int>::iterator;    // not const_iterator as std::partition may change the iterator's value
using namespace std::chrono_literals; 
using namespace std::chrono;

// Questions: 1. does beg change? Yes, it keeps track of the element, because internally std::partition moves iterators. 
void quick_sort(It beg, It end, std::list<int>& ls3){
    auto pivot_it = beg;      // the name pivot_it, divide_point: really helped debugging! So it may be worth it to create copies when writing. 
    int pivot = *pivot_it; 
    auto new_beg = beg; 
    ++new_beg;
    if (beg == end || new_beg == end) return;    //biggest challenge is you have to make copy of iterators!
    
    auto divide_point = std::partition(new_beg, end, [&pivot](const int& i){return i < pivot;});    // if everything is smaller, then divide point will be end! 
    
    ls3.splice(pivot_it, ls3, new_beg, divide_point);    // We have to splice this list, as partition just gives a divide, no gap in between.

    if (*new_beg < *pivot_it)    // Most Tricky Bug: *new_beg is larger than *pivot_it, if everything is larger than pivot_it
        quick_sort(new_beg, pivot_it, ls3);    // Because iterator tracks element, we know we this is the beginning 
    quick_sort(divide_point, end, ls3);        // divide point can't be after end
}

void quick_sort_async(It beg, It end, std::list<int>& ls3){
  auto new_beg = beg; 
  ++new_beg; 
   if (beg == end || new_beg == end) return;  
   int pivot = *beg; 

   auto divide_point = std::partition(new_beg, end, [&pivot](const int& i){return i < pivot;}); 
   auto lower_part_beg = beg;       // in the previous implementation, this might be a bug
   ++lower_part_beg; 
   ls3.splice(beg, ls3, lower_part_beg, divide_point);    // if lower_part_beg == divide_point, that's fine

   std::future<void> future; 
   if (*lower_part_beg < *beg)
      future = std::async(&quick_sort_async, lower_part_beg, beg, std::ref<std::list<int>>(ls3));
     // quick_sort_async(lower_part_beg, beg, ls3); 
   quick_sort_async(divide_point, end, ls3);
   if (future.valid())
   future.get(); 
}

int main()
{
  // // 1
  // std::list<int> ls {1,2,3,4,5}; 
  // std::list<int> ls2 {6,7,8}; 
  // ls.splice(ls.end(), ls2);    //Trap: this only moves single element ls2.begin() to list. 
  // for (auto l:ls) cout<<l<<endl;
  //
  // // 2 partition
  // int pivot = 4;
  // auto first_it = std::partition(ls.begin(), ls.end(), [&pivot](const int& i){return i < pivot; }); 
  // cout<<"first it: "<<*first_it<<endl;

  // 3 quick sort and test framework
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_int_distribution<> distrib(1, 100);

  duration<double> async_time, serial_time; 

  for (int i = 0; i < 40; ++i){
      //build random vector
      std::list<int> ls3;
      for (int j = 0; j <= i; ++j){
        ls3.push_back(distrib(gen));
      }

      time_point<system_clock> t1 = system_clock::now();
      quick_sort_async(ls3.begin(), ls3.end(), ls3); 
      time_point<system_clock> t2 = system_clock::now();
      quick_sort(ls3.begin(), ls3.end(), ls3); 
      time_point<system_clock> t3 = system_clock::now();

      //timing
      async_time += (t2 - t1);
      serial_time += (t3 - t2);

      //testing 
      int prev = INT_MIN;
      for(int l : ls3){
        if (l < prev) assert(false);
        prev = l;
      }
  }
  cout<<"tests all good!"<<endl;
  cout<<"async_time: "<<async_time.count()<<" | serial time: "<<serial_time.count()<<endl;
  
}
