#include <queue>
#include <iostream>

using std::cout; using std::endl; 

// case 2: you have a pair
struct compare{
    bool operator ()(const std::pair<int, int> p1, const std::pair<int, int> p2){
        return p1.first < p2.first; 
    }
}; 

int main()
{
  // pq is a "container adapter", i.e., on top of an existing container like std::vector or deque
  // allows duplicates, which is like std::multiset
  // O(log(n)) Internally this is a heap
  // Good explanation: http://pages.cs.wisc.edu/~vernon/cs367/notes/11.PRIORITY-Q.html
  std::priority_queue<int, std::vector<int>> pq; // by default, top of the pq is the greatest
  
  std::vector<int> vec = {24, 3, 15, 26, 38}; 
  for (unsigned int i = 0; i < vec.size(); ++i) {
    pq.push(vec.at(i)); 
  }

  while (!pq.empty()){
    cout<<pq.top()<<endl; 
    pq.pop(); 
  }

    // case 2: you have a pair. Now you have 3 methods: 
    //  1. overload >, (if it's custom) 
    //  2. define lambda. But this 
    //  3. get struct (you don't need ())
    // But anyway. a>b makes ascending order
    auto comp2 = [](const std::pair<int, int> p1, const std::pair<int, int> p2){ return p1.first > p2.first;}; 
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, decltype(comp2)> pq2(comp2); 
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, compare> pq3; 
    std::vector<std::pair<int, int>> vec2 {{1,2}, {3,4}, {5,6}}; 
    cout<<"pq2"<<endl;
    for (const auto& p: vec2){
        pq2.push(p); 
    }
    while(!pq2.empty()){
        auto p = pq2.top(); 
        pq2.pop(); 
        cout<<p.first<<", "<<p.second<<endl;
    }

    cout<<"pq3"<<endl;
    for (const auto& p: vec2){
        pq3.push(p);
    }
    while(!pq3.empty()){
        auto p = pq3.top();
        pq3.pop();
        cout<<p.first<<", "<<p.second<<endl;
    }

}
