#include <mutex>
#include <memory>
#include <condition_variable>
#include <iostream>

#include <chrono>
#include<thread>

using namespace std::chrono; 
using namespace std::chrono_literals;

using std::cout; using std::endl; 

/**
* This fine grained thread safe queue is based on singly lined list. The tail always points to a dummy node, so the locking times will be reduced.
* The reason is without the dummy node, when there's only one node, push and pop will modify the same underlying data, which requires locking two locks at the same time. That eventually is not fine-grained. 
*/

template<typename T>
class ThreadSafeQueue
{
  struct Node {   // we don't make it a separate template pointer because this is more concise. 
    std::shared_ptr<T> data_ = nullptr;   // Because push will use shared_ptr as well.
    std::unique_ptr<Node> next_ = nullptr;    // next_ manages next node's lifetime, because this is the queue's own business
  };

public:
  ThreadSafeQueue ();
  ~ThreadSafeQueue ();
  // Using std::shared_ptr because of its flexibility of memory management - allows the user to push a pointer to something that's currently somewhere else. can be easily converted to std::unique_ptr. If pushing T, then move ctor is needed.
  void push(std::shared_ptr<T>data); 
  std::shared_ptr<T> try_pop() noexcept;
  std::shared_ptr<T> wait_and_pop() noexcept;
  bool empty() const noexcept; 
private:
  std::unique_ptr<Node> head_; 
  Node* tail_;    // raw pointer because tail_ is not in charge of node's lifetime; instead next_ or head_ are. 
  ThreadSafeQueue(const ThreadSafeQueue&) = delete;
  ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

  std::mutex head_mtx_; 
  std::mutex tail_mtx_; 
  std::condition_variable cv_; 
};

template<typename T>
ThreadSafeQueue<T>::ThreadSafeQueue() : 
  head_(new Node()), tail_(head_.get()) {
}

template<typename T>
ThreadSafeQueue<T>::~ThreadSafeQueue() = default; 

template<typename T>
void ThreadSafeQueue<T>::push(std::shared_ptr<T>data){
   // Note: passing in a copy of data, small redundancy for the reference count.
   // Also may throw exception when data is created. But memory will be freed.  
   {
       auto new_node = std::unique_ptr<Node>(new Node());   // do this outside of the lock
       std::lock_guard<std::mutex> lg(tail_mtx_);
       tail_ -> data_ = data; 
       tail_->next_ = std::move(new_node);
       tail_ = tail_->next_.get();
   }
   cv_.notify_one();
}

template<typename T>
std::shared_ptr<T> ThreadSafeQueue<T>::try_pop()noexcept{
   std::unique_ptr<Node> temp; 
   {
       std::lock_guard<std::mutex> lg_head(head_mtx_);   // against other pop calls
       {
         std::lock_guard<std::mutex> lg_tail(tail_mtx_);  // against push calls
         if (head_.get() == tail_) return nullptr; 
       }
      // because tail_ will never touch head when it's not empty, we don't need a lock here!
       temp = std::move(head_); 
       head_ = std::move(temp->next_);
   }
   return temp->data_; // temp is destroyed here, which might be expensive. But it's outside any lock!
}

template<typename T>
std::shared_ptr<T> ThreadSafeQueue<T>::wait_and_pop() noexcept{
  // Note: since this function doesn't hold head_mtx_ forever, try_pop will be wait-free.
  // TODO However, this function still needs an atomic variable (like a count) to inform the queue that it's done sleeping.
    std::shared_ptr<T> ret;
    {
      std::unique_lock<std::mutex> ul_head(head_mtx_);   // against other pop calls
      // wait on head_mtx_ because we just need tail_mtx_ for empty check
      cv_.wait(ul_head, [&ret, this](){
         // get tail. If failed, will keep waiting 
          std::lock_guard<std::mutex> lg_tail(tail_mtx_);
          return (this->head_.get() == this->tail_)? false : true;
      });
      // pop_head while ul_head is still locked
      auto temp = std::move(this->head_);
      head_ = std::move(temp->next_);
      ret = temp->data_;
    }
    return ret; 
}

template<typename T>
bool ThreadSafeQueue<T>::empty() const noexcept{
     std::lock_guard<std::mutex> lg_head(head_mtx_);   // against other pop calls
     std::lock_guard<std::mutex> lg_tail(tail_mtx_);  // against push calls
     return (head_.get() == tail_);
}

ThreadSafeQueue<int> q; 
void push_test(){
  std::this_thread::sleep_for(1s);
  q.push(std::make_shared<int>(300)); 
  std::this_thread::sleep_for(1s);
  q.push(std::make_shared<int>(500)); 
}

void pop_test(){
  cout<<*(q.wait_and_pop())<<endl;
}

int main()
{
  std::thread th_a(push_test);
  std::thread th_b(push_test);
  std::thread th1(pop_test);
  std::thread th2(pop_test);
  th_a.join();
  th_b.join();
  th1.join(); 
  th2.join();
}
