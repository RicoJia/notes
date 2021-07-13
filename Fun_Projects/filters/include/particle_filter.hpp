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
      void register_belief_callback(std::function<void(std::vector<double>&)>); 

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
      std::function<void(const std::vector<double>&)> calc_observation_cb_; 

      //The callable takes in the final state estimate (belief), and stores it or does whatever.
      std::function<void(std::vector<double>&)> send_belief_cb_; 

      // generate a random number in [lower_lim, upper_lim] following universal distribution
      std::vector<double> generate_random_num_universal (const double& lower_lim, const double& upper_lim, const unsigned int& num) const ; 

      // resampling using Russian Rollet
      void resampling(); 


  }; 
}
#endif /* end of include guard: __PARTICLE_FILTER_HPP__ */
