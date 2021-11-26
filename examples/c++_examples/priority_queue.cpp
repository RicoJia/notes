#include <queue>
#include <iostream>

using std::cout; using std::endl; 

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
}
