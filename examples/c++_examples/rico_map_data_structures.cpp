// Sequence: (3, 6), (17, 15), (13, 15), (6, 12), (9, 1), (2, 7), (10, 19)
// Structure: https://www.geeksforgeeks.org/wp-content/uploads/ktree_1.png

#include <vector>
#include <Eigen/Dense>
#include <memory>

using namespace Eigen; 

namespace Util  {
struct Node {
    Node(){}
    Node(const VectorXd state, Node* parent = nullptr): state_(state), parent_(parent){}
    VectorXd state_; 
    Node* parent_ = nullptr; 
};

double best_possible_dist(const Eigen::VectorXd& target, const Eigen::VectorXd& upper_lims, const Eigen::VectorXd& lower_lims){
    VectorXd diff = Eigen::VectorXd::Zero(target.size());
    auto origin = diff; 
    for (unsigned int i = 0; i < target.size(); ++i) {
        if(lower_lims(i) < target(i) && target(i) < upper_lims(i)){
            diff(i) = 0.0; 
        }
        else{
            diff(i) = std::min(std::abs(lower_lims(i) - target(i)), std::abs((upper_lims(i) - target(i))));
        }
    }
    return (diff - origin).norm(); 
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
          find_nearest_neighbor(point, ptr_best, root_ptr, 0, VectorXd::Constant(sz, std::numeric_limits<double>::max()), VectorXd::Constant(sz, std::numeric_limits<double>::min()));
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

      //          find_nearest_neighbor(point, ptr_best, root_ptr, 0, VectorXd::Constant(sz, std::numeric_limits<double>::max()), VectorXd::Constant(sz, std::numeric_limits<double>::min()));

      void find_nearest_neighbor(const Node& point, KdTreeNode** ptr_best, KdTreeNode* node, int dim, const Eigen::VectorXd& upper_lims, const Eigen::VectorXd& lower_lims) const {
          if (node == nullptr) return;
          // So far, the best distance to the point 
          double best_dist = ((*ptr_best)->point_->state_ - point.state_).norm();
          // predict the best possible distance from the current point
          double best_possible_distance = best_possible_dist(point.state_, upper_lims,lower_lims);

          if (best_possible_distance > best_dist) return; 
          if ((point.state_ - node->point_->state_).norm() < best_dist){
              *ptr_best = node; 
          }

          int next_dim = (dim+1)%(point.state_.size());
          // keep searching the subtree in the most promising order: 
          auto right_upper_lims = upper_lims; 
          auto left_upper_lims = upper_lims; 
          auto right_lower_lims = lower_lims; 
          auto left_lower_lims = lower_lims; 

          right_lower_lims(dim) = node->point_->state_(dim); 
          left_upper_lims(dim) = node->point_->state_(dim); 

          if(point.state_(dim) < node->point_->state_(dim)){
              find_nearest_neighbor(point, ptr_best, (node->left_).get(), next_dim, left_upper_lims, left_lower_lims);
              find_nearest_neighbor(point, ptr_best, (node->right_).get(), next_dim, right_upper_lims, right_lower_lims);
          }

      }
};

}

using namespace Util;
#include <iostream>
using std::cout; using std::endl; 

int main(){
    KdTree kd_tree; 
    // Sequence: (3, 6), (17, 15), (13, 15), (6, 12), (9, 1), (2, 7), (10, 19)
    std::vector<Eigen::Vector2d> vecs{{3, 6}, {17, 15}, {13, 15}, {6, 12}, {9, 1}, {2, 7}, {10, 19}};
    for (const auto& vec: vecs){
        kd_tree.insert(std::make_unique<Node>(vec, nullptr));
    }

    // Node& x_min = kd_tree.find_min(0);
    // cout<<"x_min: "<<x_min.state_<<endl;
    // Node& y_min = kd_tree.find_min(1);
    // cout<<"y_min: "<<y_min.state_<<endl;

    std::vector<Eigen::Vector2d> min_neighbors{{2,7}, {3,6}, {6,12}, {9,1}};
    std::vector<Eigen::Vector2d> test_neighbors{{1,7}, {3,4}, {7,12}, {10,1}};
    for(const auto& item: test_neighbors){
        Node n {item, nullptr}; 
        auto nn = kd_tree.find_nearest_neighbor(n); 
        cout<<nn.state_<<endl;
    }

}


