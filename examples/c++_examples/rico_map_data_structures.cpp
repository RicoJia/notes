// Sequence: (3, 6), (17, 15), (13, 15), (6, 12), (9, 1), (2, 7), (10, 19)
// Structure: https://www.geeksforgeeks.org/wp-content/uploads/ktree_1.png
#include <iostream>
using std::cout; using std::endl; 

#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <chrono>

using namespace Eigen; 

namespace Util  {

using namespace std::chrono; 
using namespace std::chrono_literals;
struct Profiler_Timer{
   Profiler_Timer(const std::string& name = ""): name_(name){
       start_ = std::chrono::high_resolution_clock::now();  
   } 

   ~Profiler_Timer(){
      auto end = std::chrono::high_resolution_clock::now();
      auto start_us = std::chrono::time_point_cast<std::chrono::microseconds>(start_).time_since_epoch().count(); 
      auto end_us = std::chrono::time_point_cast<std::chrono::microseconds>(end).time_since_epoch().count(); 
      std::cout<<name_<<"finishes in "<< (end_us - start_us)/1000000.0<<"s"<<endl; 
   }

   std::chrono::time_point<std::chrono::high_resolution_clock> start_; 
   std::string name_; 
};

struct Node {
    Node(){}
    Node(const VectorXd state, Node* parent = nullptr): state_(state), parent_(parent){}
    VectorXd state_; 
    Node* parent_ = nullptr; 
};

// Find the shortest distance from target to the boounding box defined by upper_lims, and lower_lims
double best_possible_dist(const Eigen::VectorXd& target, const Eigen::VectorXd& upper_lims, const Eigen::VectorXd& lower_lims){
    double dist_squared = 0;
    for (unsigned int dim = 0; dim < upper_lims.size(); ++dim) {
        double closest_bound_current_dim; 
        if (lower_lims(dim) < target(dim) && target(dim) < upper_lims(dim)){
            // bounding box in the current dimension encloses the target
            closest_bound_current_dim = 0; 
        }
        else{
           // bounding box in the current dimension is outside of the target: just care about which bound is closer to the target
           closest_bound_current_dim = std::min(std::abs(lower_lims(dim) - target(dim)), std::abs(upper_lims(dim) - target(dim)));
        }

        dist_squared += std::pow(closest_bound_current_dim,2); 
    }
    
    return std::sqrt(dist_squared); 
}

class KdTree{
    using NodePtr = std::unique_ptr<Node>; 
    struct KdTreeNode{
      NodePtr point_ = nullptr;
      std::unique_ptr<KdTreeNode> left_ = nullptr;
      std::unique_ptr<KdTreeNode> right_ = nullptr;
    };

    using KdTreeNodePtr = std::unique_ptr<KdTreeNode>;
    public:
      // public functions: use Node for interfacing 
      KdTree(){}
      ~KdTree () = default;

      void insert(NodePtr point){
            insert(std::move(point), root_, 0);
      }

      Node& find_min(const int target_dim){
          if (root_ == nullptr){
            throw "root is null!";
          }
          auto min_node = find_min(root_.get(), 0, target_dim);
          return *(min_node->point_);
      }

      Node& find_nearest_neighbor(const Node& point){
          if (root_ == nullptr) throw "root is null!";
          KdTreeNode* root_ptr = root_.get();
          KdTreeNode** ptr_best = &root_ptr;
          size_t sz = root_->point_->state_.size();
          double best_dist = std::numeric_limits<double>::max(); 
          find_nearest_neighbor(point, ptr_best, best_dist, root_ptr, 0, VectorXd::Constant(sz, std::numeric_limits<double>::max()), VectorXd::Constant(sz, -1 * std::numeric_limits<double>::max()));
          return *((*ptr_best)->point_);
      }

    private:
      KdTreeNodePtr root_ = nullptr;

      // have to be KdTreeNodePtr& since we need to modify node
      void insert(NodePtr&& point, KdTreeNodePtr& kd_tree_node, int dim){
          if (kd_tree_node == nullptr){
            kd_tree_node = std::make_unique<KdTreeNode>();
            kd_tree_node->point_ = std::forward<NodePtr>(point);
          }
          else if (kd_tree_node->point_ -> state_ == point -> state_){
            kd_tree_node -> point_ = std::forward<NodePtr>(point); 
            return;
          }
          else{
              // left child
              int next_dim = (dim+1)%(point->state_.size());
              if (point->state_(dim) < kd_tree_node->point_->state_(dim)){
                  insert(std::move(point), kd_tree_node->left_, next_dim);
              }
              else{
                  insert(std::move(point), kd_tree_node->right_, next_dim);
              }
          }
      }

      KdTreeNode* find_min(KdTreeNode* node, int dim, const int target_dim){
          if(node == nullptr) return nullptr;
          int next_dim = (dim+1)%(node->point_->state_.size());
          auto get_min = [node, target_dim](KdTreeNode* n1, KdTreeNode* n2){
               KdTreeNode* min_node = node;
               if (n1 != nullptr && n1 -> point_->state_[target_dim] < min_node -> point_->state_[target_dim]) min_node = n1;
               if (n2 != nullptr && n2 -> point_->state_[target_dim] < min_node -> point_->state_[target_dim]) min_node = n2;
               return min_node;
          };

          if (dim != target_dim){
              KdTreeNode* min_1 = find_min((node->left_).get(), next_dim, target_dim);
              KdTreeNode* min_2 = find_min((node->right_).get(), next_dim, target_dim);
              return get_min(min_1, min_2);
          }
          else{
            auto subtree_min_node = find_min((node->left_).get(), next_dim, target_dim); 
            // because subtree_min_node might be nullptr too
            auto min_node = get_min(subtree_min_node, nullptr);
            return min_node;
          } 
      }


      // Update the bounding box for subtrees with the current nodes dimension. 
      void find_nearest_neighbor(const Node& point, KdTreeNode** ptr_best, double& best_dist, KdTreeNode* node, int dim, const Eigen::VectorXd& upper_lims, const Eigen::VectorXd& lower_lims) const {
          if (node == nullptr) return;

          // predict the best possible distance from the current point
          double best_dist_possible = best_possible_dist(point.state_, upper_lims,lower_lims);

          if (best_dist_possible > best_dist) return; 

          // If the current node to the point is better than best_dist
          if ((point.state_ - node->point_->state_).norm() <= best_dist){
              *ptr_best = node; 
              best_dist = ((*ptr_best)->point_->state_ - point.state_).norm();
          }

          int next_dim = (dim+1)%(point.state_.size());
          // keep searching the subtree in the most promising order: 
          auto right_upper_lims = upper_lims; 
          auto left_upper_lims = upper_lims; 
          auto right_lower_lims = lower_lims; 
          auto left_lower_lims = lower_lims; 

          right_lower_lims(dim) = node->point_->state_(dim); 
          left_upper_lims(dim) = node->point_->state_(dim); 

          find_nearest_neighbor(point, ptr_best, best_dist, (node->left_).get(), next_dim, left_upper_lims, left_lower_lims);
          find_nearest_neighbor(point, ptr_best, best_dist, (node->right_).get(), next_dim, right_upper_lims, right_lower_lims);
      }
};

}

using namespace Util;

#include <tuple>
template <typename T,
          typename TIter = decltype(std::begin(std::declval<T>())),
          typename = decltype(std::end(std::declval<T>()))
          >
constexpr auto enumerate(T&& iterable){
    struct iterator{
        size_t i;
        TIter iter;
        bool operator != (const iterator& other){return other.iter != iter; }
        void operator ++(){
            ++i; ++iter;
        }

        auto operator *(){
            return std::tie(i, *iter);
        }

    };

    struct iterable_wrapper{
        T iterable;
        auto begin(){return iterator{0, iterable.begin()}; }
        auto end(){return iterator{0, iterable.end()}; }

    };

    return iterable_wrapper{std::forward<T>(iterable)};
}


int main(){
    KdTree kd_tree; 
    // Sequence: (3, 6), (17, 15), (13, 15), (6, 12), (9, 1), (2, 7), (10, 19)
    {
        Util::Profiler_Timer pt;
        std::vector<Eigen::Vector2d> vecs{{3, 6}, {17, 15}, {13, 15}, {6, 12}, {9, 1}, {2, 7}, {10, 19}};
        for (const auto& vec: vecs){
            kd_tree.insert(std::make_unique<Node>(vec, nullptr));
        }
    }

    bool test_passed = true;

    // Node& x_min = kd_tree.find_min(0);
    // cout<<"x_min: "<<x_min.state_<<endl;
    // Node& y_min = kd_tree.find_min(1);
    // cout<<"y_min: "<<y_min.state_<<endl;

    std::vector<Eigen::Vector2d> real_neighbors{{2,7}, {3,6}, {6,12}, {9,1}, {10, 19}, {13,15}, {17, 15}};
    std::vector<Eigen::Vector2d> test_items{{1,7}, {3,4}, {7,12}, {10,1}, {10,20}, {13,14}, {17, 16}};
    // for eigen
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " [", "] ");

    Util::Profiler_Timer pt;

    for(const auto& [index, item]: enumerate(test_items)){
        Node n {item, nullptr}; 
        auto nn = kd_tree.find_nearest_neighbor(n); 
        if(nn.state_ != real_neighbors.at(index)){
            test_passed  = false; 
            cout<<"Test item: "<<item.format(CommaInitFmt)<<", should be"<<real_neighbors.at(index).format(CommaInitFmt) << "but got" << nn.state_.format(CommaInitFmt)<<endl;
        }
    }

    if (test_passed){
        cout<<"Test Passed"<<endl; 
    }

}


