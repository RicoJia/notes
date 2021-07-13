/** @file */
#include "particle_filter.hpp"
#include <random>
#include <exception>
#include <algorithm>
#include <utility>

namespace Filter{
  Particle_Filter::Particle_Filter(const std::vector<std::pair<double, double>>& ranges, const unsigned int particle_num): 
    states_(std::vector<State>(particle_num))

  {
    // generate N evenly distributed states for each state vec, N = ranges.size() 
    for (unsigned int state_i = 0; state_i < ranges.size(); ++state_i){
      auto dim = generate_random_num_universal(ranges.at(state_i).first, ranges.at(state_i).second, particle_num); 
      for (unsigned int i = 0; i < particle_num; ++i){
        states_.at(i).state_vec.emplace_back(dim.at(i)); 
        states_.at(i).weight = 1.0/particle_num; 
      }
    }

    // launch a thread pool for parallelism
    auto max_num_threads = std::thread::hardware_concurrency(); 
    auto num_threads = std::min( (max_num_threads < 2) ? 2: max_num_threads - 1, particle_num ); 
    thread_pool_ = std::make_unique<ThreadPool>(num_threads); 
  }

  void Particle_Filter::register_control_callback(std::function<void (std::vector<double>&)> update_control_cb){
    update_control_cb_ = update_control_cb; 
  }

  void Particle_Filter::register_observation_callback(std::function<double (const std::vector<double>&)> calc_observation_cb){
    calc_observation_cb_ = calc_observation_cb; 
  }

  void Particle_Filter::register_belief_callback(std::function<void (std::vector<double>&)> send_belief_cb){
    send_belief_cb_ = send_belief_cb;  
  }

  void Particle_Filter::resampling(){
    // create the CDF of weight at each state. 
    auto particle_num = states_.size(); 
    std::vector<double> weight_cdfs(particle_num, states_.at(0).weight);
    std::transform(states_.begin()+1, states_.end(), states_.begin(), weight_cdfs.begin(), [](const State& current_state, const State& last_state){return current_state.weight + last_state.weight;});

    // generate a random number in [0, 1/particle_num] , then do Russian-Roulette importance sampling 
    double r = generate_random_num_universal(0, 1.0/particle_num, 1).at(0); 
    std::vector<State> new_states;
    new_states.reserve(particle_num);
    auto cdf_index = 0;
    for(auto i = 0; i < particle_num; ++i){
      double current_spoke = r + i * 1.0 /particle_num;
      for (; cdf_index < particle_num && weight_cdfs.at(cdf_index) < current_spoke; ++cdf_index){      }
      new_states.emplace_back(State{std::move(states_.at(cdf_index - 1).state_vec), 1.0/particle_num});
    }
    states_ = std::move(new_states); 
  }

  std::vector<double> Particle_Filter::generate_random_num_universal (const double& lower_lim, const double& upper_lim, const unsigned int& num) const {
      std::random_device rd; //Will be used to obtain a seed for the random number engine
      std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
      std::uniform_int_distribution<> distribution (lower_lim, upper_lim);
      std::vector<double> ret_vec;
      ret_vec.reserve(num);
      for (auto i = 0; i < num; ++i)
        ret_vec.emplace_back(distribution(gen)); 
      return ret_vec; 
  }

  void Particle_Filter::run(){
    if (calc_observation_cb_ == nullptr){
      throw std::runtime_error("calc_observation_cb_ has not been attached. Particle Filter not running"); 
    }

    if (send_belief_cb_ == nullptr){
      throw std::runtime_error("send_belief_cb_ has not been attached. Particle Filter not running");
    }
    
    //TODO: to test
    resampling(); 
  }

}
