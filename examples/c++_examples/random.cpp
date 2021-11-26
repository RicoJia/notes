#include <cmath>
#include <vector>
#include <random>
#include <chrono>
#include <iostream>
using std::cout; using std::endl; 

std::vector<double> generate_random_num_gaussian(const double& mean, const double& std_dev, const unsigned int& num){
    // construct a trivial random generator engine from a time-based seed:
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);

  std::normal_distribution<double> distribution(mean, std_dev);
  std::vector<double> ret; 
  ret.reserve(num); 
  for (unsigned int i = 0; i < num; ++i){
    ret.emplace_back(distribution(generator)); 
  }
  return ret; 
}  

void generate_uniform_dist_int(){
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(1,6); // distribution in range [1, 6]
    // std::uniform_real_distribution for float
    // std::uniform_real_distribution<double> x(lower_bound_corner_(0), upper_bound_corner_(0));
    std::cout << dist6(rng) << std::endl;
}
int main(){
  // for (int i = 0; i < 10; ++i){
  //   auto ran_vec = generate_random_num_gaussian(0.0, 0.1, 1); 
  //   for (auto i : ran_vec) cout<<i<<" "<<endl;
  // }
  generate_uniform_dist_int();
}
