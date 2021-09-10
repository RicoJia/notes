#include <algorithm>
#include <utility>
#include <vector>
#include <list>
#include <functional>   //std::hash
#include <shared_mutex>
#include <string>

// Notes
//  1. LookupTable has a fixed number of buckets. Each bucket will hold a std::list of members
//  2. std::hash will transform key to a number. Works well with prime num?

template<typename K, typename V>
class LookupTable
{
private:
  struct Bucket{
      private:
          using Pair = std::pair<K,V>; 
          std::list<Pair> pairs_;
          std::mutex mtx_;
          typename std::list<Pair>::iterator find_pair(const K& key){
              return std::find_if(pairs_.begin(), pairs_.end(), [&key](const Pair& p){return p.first == key;}); 
          }

      public:
        V get_value(const K& key){
          std::shared_lock sl(mtx_);     // c++17
          auto pair_it = find_pair(key); 
          if (pair_it == pairs_.end()) throw "Pair not found!"; 
          return pair_it->second; 
        }

        void insert_or_modify(const K& key){

        }
  };

  std::vector<Bucket> buckets_;

  Bucket& get_bucket(const K& key){
    // std::hash<std::string>("lol"); 
    return buckets_[0];
    // return buckets_[(std::hash<K>(key))%(buckets_.size())]; 
  }

public:
  LookupTable (size_t bucket_num) : buckets_(bucket_num){
  }
  ~LookupTable () = default;

  V get_value(const K& key){
      return get_bucket(key).get_value(key);
  }

  void operator[](const K& key){
      
  }
};


int main()
{
  LookupTable<std::string, double> table(19);     //std::hash works best with prime number
  table.get_value("hehe");
      
}
