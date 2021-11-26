// KD tree: time complexity o(log(n)) for insert, remove, lookup 
#include <numeric>
#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <climits>
#include <cmath>
#include <limits>

using std::cout; using std::endl;

// The cutting dimension starts at the first dimension in coord.

// length check to be added. Now we assume the user will have the points of the same dimension .
using Point = std::vector<double>;
void print_point(const Point& p){
    cout<<"("; 
    for (auto i: p) cout<<i<<" "; 
    cout<<")";
}

double dist(const Point& p1, const Point& p2){
    Point diff = p1; 
    std::transform(p1.begin(), p1.end(), p2.begin(), diff.begin(), [](double i1, double i2){return i1 - i2; }); 
    return sqrt(std::accumulate(diff.begin(), diff.end(), 0.0, [](double ss, double i){return ss + i*i;}));
}

double best_possible_dist(const Point& target, const Point& upper_lims, const Point& lower_lims){
    auto diff = Point(target.size(), 0.0);
    auto origin = diff; 
    for (unsigned int i = 0; i < target.size(); ++i) {
        if(lower_lims.at(i) < target.at(i) && target.at(i) < upper_lims.at(i)){
            diff.at(i) = 0.0; 
        }
        else{
            diff.at(i) = std::min(std::abs(lower_lims.at(i) - target.at(i)), std::abs((upper_lims.at(i) - target.at(i))));
        }
    }
    return dist(diff, origin); 
}

template<typename T>
class KdTree
{
  struct Node{
    T point_;
    std::unique_ptr<Node> left_;
    std::unique_ptr<Node> right_;
  };
using NodePtr = std::unique_ptr<Node>;
public:
  KdTree ();
  ~KdTree ();

  void insert(T point);
  //TODO
  void print_tree();

  T find_min(const int target_dim);
  bool remove_point(const T& point);

  T find_nearest_neighbor(const T& point);
private:
  NodePtr root_;
  // have to be NodePtr& since we need to modify node
  void insert(T point, NodePtr& node, int dim){
    if (node == nullptr){
      node = std::make_unique<Node>();
      node->point_ = point;
    }
    else if (node->point_ == point){
      return;
    }
    else{
      // left child
      int next_dim = (dim+1)%(point.size());
      if (point.at(dim) < node->point_.at(dim)){
          insert(point, node->left_, next_dim);
      }
      else{
          insert(point, node->right_, next_dim);
      }
    }
  }

  Node* find_min(Node* node, int dim, const int target_dim){
    if(node == nullptr) return nullptr;

    int next_dim = (dim+1)%(node->point_.size());
    auto get_min = [node, target_dim](Node* n1, Node* n2){
         Node* min_node = node;
         if (n1 != nullptr && n1 -> point_[target_dim] < min_node -> point_[target_dim]) min_node = n1;
         if (n2 != nullptr && n2 -> point_[target_dim] < min_node -> point_[target_dim]) min_node = n2;
         return min_node;
    };

    if (dim != target_dim){
        Node* min_1 = find_min((node->left_).get(), next_dim, target_dim);
        Node* min_2 = find_min((node->right_).get(), next_dim, target_dim);
        return get_min(min_1, min_2);
    }
    else{
      auto subtree_min_node = find_min((node->left_).get(), next_dim, target_dim); 
      // because subtree_min_node might be nullptr too
      auto min_node = get_min(subtree_min_node, nullptr);
      return min_node;
    }
  }

  //TODO: optional
  Node* remove_point(const T& point, Node* node, int dim){
      // This only happens when root == nullptr. 
      if (node == nullptr) return nullptr; 
        
      int next_dim = (dim+1)%(node->point_.size());

      // if we have found our node to remove: 
      if (node -> point_ == point){
          // when we have a right part, we find the min of the right part to substitute
          if (node -> right_ != nullptr){
              Node* min_right_part = find_min(node->right_, dim, next_dim); 
              node -> point_ = min_right_part -> point_; 
              // node_->right_ = remove_point(node->point_, node->right_, next_dim); 
          }
          // we gotta reach the bottom so we're sure no more subtree to trim!

      }
      else{

      }
  }

  void find_nearest_neighbor(const T& point, Node** ptr_best, Node* node, int dim, Point upper_lims, Point lower_lims){
      if (node == nullptr) return;
      double best_dist = dist((*ptr_best)->point_, point);
      double best_possible_distance = best_possible_dist(point, upper_lims,lower_lims);
      if (best_possible_distance > best_dist) return; 
      if (dist(point, node->point_) < best_dist){
          *ptr_best = node; 
      }
      
      int next_dim = (dim+1)%(point.size());
      // keep searching the subtree in the most promising order: 
      Point right_upper_lims = upper_lims; 
      Point left_upper_lims = upper_lims; 
      Point right_lower_lims = lower_lims; 
      Point left_lower_lims = lower_lims; 

      right_lower_lims.at(dim) = node->point_.at(dim); 
      left_upper_lims.at(dim) = node->point_.at(dim); 

      if(point.at(dim) < node->point_.at(dim)){
        find_nearest_neighbor(point, ptr_best, (node->left_).get(), next_dim, left_upper_lims, left_lower_lims);  
        find_nearest_neighbor(point, ptr_best, (node->right_).get(), next_dim, right_upper_lims, right_lower_lims);  
      }
      else{
        find_nearest_neighbor(point, ptr_best, (node->right_).get(), next_dim, right_upper_lims, right_lower_lims);  
        find_nearest_neighbor(point, ptr_best, (node->left_).get(), next_dim, left_upper_lims, left_lower_lims);  
      }
  }
};

template<typename T>
KdTree<T>::KdTree(): root_(nullptr){
}

template<typename T>
KdTree<T>::~KdTree() = default;

template<typename T>
void KdTree<T>::insert(T point){
    insert(point, root_, 0);
}

template<typename T>
bool KdTree<T>::remove_point(const T& point){
  Node* ret_node = remove(point, root_, 0);
  return (ret_node == nullptr); 
}

template<typename T>
T KdTree<T>::find_min(const int target_dim){
    if (root_ == nullptr){
      throw "root is null!";
    }
    auto min_node = find_min(root_.get(), 0, target_dim);
    return min_node->point_;
}


template<typename T>
T KdTree<T>::find_nearest_neighbor(const T& point){
    if (root_ == nullptr) throw "root is null!"; 
    Node* root_ptr = root_.get();
    Node** ptr_best = &root_ptr; 
    size_t sz = root_->point_.size();
    find_nearest_neighbor(point, ptr_best, root_ptr, 0, Point(sz, std::numeric_limits<double>::max()), Point(sz, std::numeric_limits<double>::min()));
    return (*ptr_best)->point_; 
}

enum {
  X=0,
  Y=1
};


int main()
{
  KdTree<Point> kd_tree;
  kd_tree.insert({1.0, 2.0});
  kd_tree.insert({2.0, 0.0});
  kd_tree.insert({3.0, -7.0});
  kd_tree.insert({-4.0, -1.0});
  kd_tree.insert({0.0, -1.0});
  kd_tree.insert({-1.0, -2.0});

  // auto point = kd_tree.find_min(X);
  auto point = kd_tree.find_nearest_neighbor({3.0, -100.0}); 
  print_point(point); 

}

