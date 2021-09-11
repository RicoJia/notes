#include <iostream>
#include <algorithm>
#include <mutex>
#include <utility>
#include <vector>
#include <list>
#include <functional>   //std::hash
#include <shared_mutex>
#include <string>
#include <thread>
using std::cout; using std::endl; 

// Notes
//  1. LookupTable has a fixed number of buckets. Each bucket will hold a std::list of members
//  2. std::hash will transform key to a number. Works well with prime num?

template<typename K, typename V>
class LookupTable
{
private:
  using Pair = std::pair<K,V>; 
  struct Bucket{
      private:
          std::list<Pair> pairs_;
          std::shared_mutex mtx_;
          // const function only returns const_iterator.
          typename std::list<Pair>::iterator find_pair(const K& key){
              // return std::find_if(pairs_.cbegin(), pairs_.cend(), [&key](const Pair& p){return p.first == key;}); 
              return std::find_if(pairs_.begin(), pairs_.end(), [&key](const Pair& p){return p.first == key;}); 
          }

      public:
        // not const since we change mtx_
        V get_value(const K& key) {
          std::shared_lock<std::shared_mutex> sl(mtx_);     // c++17
          auto pair_it = find_pair(key); 
          if (pair_it == pairs_.end()) throw "Pair not found!"; 
          return pair_it->second; 
        }

        V& operator[](const K& key){
            std::lock_guard<std::shared_mutex> lg(mtx_); 
            auto pair_it = find_pair(key); 
            if(pair_it == pairs_.end()){
              // NOTE: trick here: A map, set needs a default value for returning V& in []
               pairs_.emplace_back(std::pair<K, V>(key, V{}));
               return pairs_.back().second; 
            }
            else{
               return pair_it->second; 
            }
        }

        void erase(const K& key){
            std::lock_guard<std::shared_mutex> lg(mtx_); 
            auto pair_it = find_pair(key);
            if (pair_it != pairs_.end()) pairs_.erase(pair_it); 
        }

  };

  std::vector<Bucket> buckets_;
  std::hash<K> hasher_;

  Bucket& get_bucket(const K& key){
    size_t index =hasher_(key) %(buckets_.size());
    return buckets_[index]; 
  }

public:
  LookupTable (size_t bucket_num) : buckets_(bucket_num), hasher_(){
  }
  ~LookupTable () = default;

  V get_value(const K& key){
      return get_bucket(key).get_value(key);
  }

  V& operator[](const K& key){
     return get_bucket(key)[key]; 
  }

  // Note: map::erase will return map::iterator pointing to the next element. One tricky thing is: it returns the iterator pointing to the next element in the bucket
  void erase(const K& key){
      get_bucket(key).erase(key); 
  }
};

//Note: std::hash works best with prime number
LookupTable<std::string, double> table(19);     

void set_and_get(double i){
    table["hehe"] = i;
    cout<<"hehe"<<endl;
    cout<<table.get_value("hehe")<<endl;
}

int main()
{
  std::vector<std::thread> th_vec; 
  th_vec.emplace_back(std::thread(set_and_get, 1));
  th_vec.emplace_back(std::thread(set_and_get, 2));
  table.erase("hehe"); 
  cout<<table["hehe"]<<endl;    // default value
  for (auto& th: th_vec) th.join(); 
}
