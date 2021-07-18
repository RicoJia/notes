/*
 * Copyright (c) 2021 Rico Studio
 * All rights reserved.
 * License: BSD-0
 * author: Rico Jia
 */
/** @file */
#ifndef __PARTICLE_FILTER_HPP__
#define __PARTICLE_FILTER_HPP__

#include <vector>
#include <deque>
#include <condition_variable>
#include <utility>
#include <functional>
#include <numeric>
#include <random>
#include <exception>
#include <algorithm>
#include <thread>
#include <utility>
#include <iostream>
#include <memory>

#include "thread_pool.hpp"

namespace Filter{
  class Particle_Filter{
    public:

      /**
      * @brief: Constructor for particle filter
      * @param: ranges - upper and lower ranges limits of each state. 
      */
      Particle_Filter(const std::vector<std::pair<double, double>>& ranges, const unsigned int particle_num); 

      /**
      * @brief: Register function for send_relief callback, which updates the state with control input in place. See Particle_Filter::update_control_cb_;
      * @param: Callable with signature void (std::vector<double>&). 
      */
      void register_control_callback(std::function<void(std::vector<double>&)>); 

      /**
      * @brief: Register function for calc_observation callback. See Particle_Filter::calc_observation_cb_
      * @param: Callable with signature double (std::vector<double>&). 
      */
      void register_observation_callback(std::function<double (const std::vector<double>&)>); 

      /**
      * @brief: Register function for send_relief callback. See Particle_Filter::send_belief_cb_
      * @param: Callable with signature void (const std::vector<double>&). 
      */
      void register_belief_callback(std::function<void(const std::vector<double>&)>); 

      /**
      * @brief: Main function that contains the main prediction-corrrection loop. 
      * @notes: std::runtime_error will be thrown if callbacks are not attached. 
      */
      void run(); 
      
    private:
      struct State{
        std::vector<double> state_vec;
        double weight;
      };
      std::vector<State> states_;

      std::condition_variable observation_ready_; 
      std::unique_ptr<ThreadPool> thread_pool_; 

      std::function<void( std::vector<double>&)> update_control_cb_;
      
      //The callable takes in a single predicted state before an observation, and returns the likelihood of observation given the state observation.
      std::function<double (const std::vector<double>&)> calc_observation_cb_; 

      //The callable takes in the final state estimate (belief), and stores it or does whatever.
      std::function<void(const std::vector<double>&)> send_belief_cb_; 

      // generate a random number in [lower_lim, upper_lim] following universal distribution
      std::vector<double> generate_random_num_universal (const double& lower_lim, const double& upper_lim, const unsigned int& num) const ; 

      // resampling using Russian Rollet
      void resampling(); 

      // return the average over each dimension 
      std::vector<double> average_belief(); 
  }; 

inline Particle_Filter::Particle_Filter(const std::vector<std::pair<double, double>>& ranges, const unsigned int particle_num): 
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

inline void Particle_Filter::register_control_callback(std::function<void (std::vector<double>&)> update_control_cb){
    update_control_cb_ = update_control_cb; 
  }

inline void Particle_Filter::register_observation_callback(std::function<double (const std::vector<double>&)> calc_observation_cb){
    calc_observation_cb_ = calc_observation_cb; 
  }

inline void Particle_Filter::register_belief_callback(std::function<void (const std::vector<double>&)> send_belief_cb){
    send_belief_cb_ = send_belief_cb;  
  }

inline void Particle_Filter::resampling(){
    // create the CDF of weight at each state. 
    auto particle_num = states_.size(); 
    std::vector<double> weight_cdfs(particle_num, states_.at(0).weight);
    std::transform(states_.begin()+1, states_.end(), weight_cdfs.begin(), weight_cdfs.begin()+1, [](const State& current_state, const double& last_total_weight){return current_state.weight + last_total_weight;});

    for (auto& i : weight_cdfs) std::cout<<i<<std::endl;
    // generate a random number in [0, 1/particle_num] , then do Russian-Roulette importance sampling 
    double r = generate_random_num_universal(0, 1.0/particle_num, 1).at(0); 
    std::vector<State> new_states;
    new_states.reserve(particle_num);
    auto cdf_index = 0;
    for(int i = 0; i < particle_num; ++i){
      double current_spoke = r + (i * 1.0) /particle_num;

      for (; cdf_index < particle_num && weight_cdfs.at(cdf_index) < current_spoke; ++cdf_index){      }
      new_states.emplace_back(State{std::move(states_.at(cdf_index).state_vec), 1.0/particle_num});
    }
    states_ = std::move(new_states); 
  }

inline std::vector<double> Particle_Filter::average_belief(){
    auto num_dim = states_.at(0).state_vec.size(); 
    std::vector<double> avg (num_dim);
    for (auto i = 0; i < num_dim; ++i){
        avg.at(i) = std::accumulate(states_.begin(), states_.end(), 0.0, [&i](double sum, const State& s1){return sum + s1.weight * s1.state_vec.at(i);});
    }
    return avg;
  }

inline std::vector<double> Particle_Filter::generate_random_num_universal (const double& lower_lim, const double& upper_lim, const unsigned int& num) const {
      std::random_device rd; //Will be used to obtain a seed for the random number engine
      std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
      std::uniform_real_distribution<> distribution (lower_lim, upper_lim);
      std::vector<double> ret_vec;
      ret_vec.reserve(num);
      for (auto i = 0; i < num; ++i)
        ret_vec.emplace_back(distribution(gen)); 
      return ret_vec; 
  }


inline void Particle_Filter::run(){
    if (update_control_cb_ == nullptr){
      throw std::runtime_error("update_control_cb_ has not been attached. Particle Filter not running");
    }

    if (calc_observation_cb_ == nullptr){
      throw std::runtime_error("calc_observation_cb_ has not been attached. Particle Filter not running");
    }

    if (send_belief_cb_ == nullptr){
      throw std::runtime_error("send_belief_cb_ has not been attached. Particle Filter not running");
    }

    //TODO: to test
    resampling();
    for (auto& state : states_)
        update_control_cb_(state.state_vec);
    for (auto& state : states_)
        calc_observation_cb_(state.state_vec);

    // normalize the states
    double sum = std::accumulate(states_.begin(), states_.end(), 0.0, [](double sum, const State& s){return sum + s.weight;});
    std::for_each(states_.begin(), states_.end(),[sum](State& s){s.weight /= sum;});

    //send average belief
    send_belief_cb_(average_belief());
  }

}
#endif /* end of include guard: __PARTICLE_FILTER_HPP__ */
