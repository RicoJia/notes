#ifndef __FILTERS_UTIL_HPP__
#define __FILTERS_UTIL_HPP__

#include <cmath>
#include <vector>
#include <random>
#include <chrono>


namespace Filter{
  namespace Util{
      // generate a random number in [lower_lim, upper_lim] following universal distribution
    std::vector<double> generate_random_num_universal (const double& lower_lim, const double& upper_lim, const unsigned int& num) {
      std::random_device rd; //Will be used to obtain a seed for the random number engine
      std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
      std::uniform_real_distribution<> distribution (lower_lim, upper_lim);
      std::vector<double> ret_vec;
      ret_vec.reserve(num);
      for (auto i = 0; i < num; ++i)
        ret_vec.emplace_back(distribution(gen));
      return ret_vec;
    }

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

    bool is_equal(double a, double b){
      return std::abs(a-b) < std::pow(10, -6); 
    }
  }
}

#endif /* end of include guard: __FILTERS_UTIL_HPP__ */
