#include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>

using std::cout; using std::endl; 

//Note: singly linked list. Essense is lock the whole head, lock each node one by one, till you get to the node
// Makes sense because all functions follow this
// Disadvantages: 1. too many mutexes!

template<typename T>
class List
{
  private:
    struct Node{
        Node(const T& val): val_(std::make_shared<T>(val)){}
        Node() = default;
        std::shared_ptr<T> val_; 
        std::unique_ptr<Node> next_;
        std::shared_mutex sh_mtx_;
    }; 

    std::unique_ptr<Node> head_; 

  public:
    // there has to be a node at the beginning, otherwise when there's no node, we don't have a mutex to lock!
    List (): head_(std::make_unique<Node>()){
    }

    ~List () = default;

    // always have a head
    void push_front(const T& val){
      auto new_node = std::make_unique<Node>(val);
      std::lock_guard<std::shared_mutex> lg(head_->sh_mtx_);
      new_node->next_ = std::move(head_->next_); 
      head_->next_ = std::move(new_node); 
    } 

    // Providing for_each as we can't lock STL iterators for getting into a node
    template<typename Predicate>
    void for_each(Predicate p){
        Node* node = head_.get(); 
        std::unique_lock<std::shared_mutex> ul(head_->sh_mtx_); 
        while(node = node->next_.get())
        {
            std::unique_lock<std::shared_mutex> next_ul(node->sh_mtx_); 
            ul.unlock(); 
            p(*(node->val_));
            ul = std::move(next_ul); 
        }
    }

    // we don't have iterator here so let's return the shared_ptr
    template<typename Predicate>
    std::shared_ptr<T> find_first_of(Predicate p){
        Node* node = head_.get(); 
        std::unique_lock<std::shared_mutex> ul(head_->sh_mtx_); 
        while(node = node->next_.get())
        {
            std::unique_lock<std::shared_mutex> next_ul(node->sh_mtx_); 
            ul.unlock(); 
            // last node will have nullptr
            if(p(*(node->val_))){
              return node->val_;
            }
            ul = std::move(next_ul); 
        } 
        return nullptr;
    }

    template<typename Predicate>
    void remove_if(Predicate p){
        Node* node = head_.get(); 
        std::unique_lock<std::shared_mutex> ul(head_->sh_mtx_); 
        while(Node*next_node = node->next_.get())
        {
            std::unique_lock<std::shared_mutex> next_ul(next_node->sh_mtx_); 
            if (p(*(next_node->val_))){
                auto old_next = std::move(next_node);
                node->next_ = std::move(old_next->next_);
                next_ul.unlock();   // since we have acquired ul and next_ul, nobody is trying to acquire next_ul now. 
            }
            else{
                ul.unlock(); 
                node = node->next_.get(); 
                ul = std::move(next_ul); 
            }
        }
    }

};

int main()
{
    List<int> ls;  
    // ls.push_front(3);
    // ls.push_front(1);
    ls.push_front(2);
    ls.remove_if([](int i){return i==2;});
    auto ptr = ls.find_first_of([](int i){return i==2;});
    cout<<*ptr<<endl;
}
