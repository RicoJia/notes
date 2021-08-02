#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <memory>
#include <vector>
#include <tuple>
#include "particle_filter.hpp"
#include <cmath>

#pragma GCC visibility push(hidden)
namespace py = pybind11;

using Filter::Particle_Filter; 
//TODO
using std::cout; using std::endl; 

#define RIGHT_SHIFT 5   // We take the first 3 digits of a pixel value.

enum State_Index{
  X = 0, 
  Y = 1, 
  VX = 2, 
  VY = 3,
  HX = 4,
  HY = 5,
  AT_DOT = 6
}; 

class FaceTrackerPF{
  public: 
    explicit FaceTrackerPF(const py::dict& inputs); 
    py::array_t<double> run_one_iteration(const py::array_t<double>& frame);

  protected: 
    //takes in a set of states, and apply control with noise.
    void control_callback(std::vector<double>& states);
    // Given state estimate, return the likelihood of the current observation
    double observation_callback(const std::vector<double>& state_estimate);
    void adjust_belief(std::vector<double>& belief);

    void calc_region_hist (std::vector<double>& hist, uint32_t x, uint32_t y, uint32_t w, uint32_t h);

    double calc_bhattacharya_coefficient(const std::vector<double>& hist1, const std::vector<double>& hist2);

    std::unique_ptr<Particle_Filter> pf_; 
    // pre-allocating return state so it's fast
    py::array_t<double> return_state_;    
    
    std::vector<std::pair<double, double>> ranges_vec_; 
    std::vector<double> noise_vec_; 
    const uint32_t particle_num_; 
    const double scale_change_disturb_; 
    const double velocity_disturb_; 
    const double delta_t_; 

    uint8_t* image_; 
    std::vector<double> roi_dist_; 
}; 

inline FaceTrackerPF::FaceTrackerPF(const py::dict& inputs) : 
    particle_num_(inputs["PARTICLE_NUM"].cast<uint32_t>()), 
    scale_change_disturb_(inputs["SCALE_CHANGE_DISTURB"].cast<double>()),
    velocity_disturb_(inputs["VELOCITY_DISTURB"].cast<double>()), 
    delta_t_(1.0/inputs["FRAME_RATE"].cast<float>()), 
    image_(nullptr) 
{
    // initialize ranges, ranges is [(upper_lim, lower_lim, standard_deviation_of_noise), ...]
    auto ranges = inputs["ranges"].cast<py::list>(); 
    for(const auto& item : ranges){
      auto range = item.cast<py::list>();
      ranges_vec_.emplace_back(std::make_pair<double, double>(range[0].cast<double>(), range[1].cast<double>())); 
      noise_vec_.emplace_back(range[2].cast<double>());   
    }
  
    // initialize pf_
    pf_ = std::make_unique<Particle_Filter>(ranges_vec_, particle_num_); 
    pf_ -> register_control_callback(std::bind(&FaceTrackerPF::control_callback, this, std::placeholders::_1)); 
    pf_ -> register_observation_callback(std::bind(&FaceTrackerPF::observation_callback, this, std::placeholders::_1));

    // initialize ROI distributions
    auto corner_points = inputs["ROI_corner_points"].cast<py::list>(); 
    auto corner_point_1 = corner_points[0].cast<std::tuple<int64_t, int64_t>>();
    auto corner_point_2 = corner_points[1].cast<std::tuple<int64_t, int64_t>>();
    std::get<0>(corner_point_1)+std::get<0>(corner_point_2);    //TODO: not working here.
    auto x_val = (std::get<0>(corner_point_1)+std::get<0>(corner_point_2))/2; 
    auto y_val = (std::get<1>(corner_point_1)+std::get<1>(corner_point_2))/2; 
    auto w = std::abs(std::get<0>(corner_point_1) - std::get<0>(corner_point_2));
    auto h = std::abs(std::get<1>(corner_point_1) - std::get<1>(corner_point_2)); 
    
    // calculate the histogram of the ROI
    auto frame = inputs["initial_frame"].cast<py::array_t<uint8_t>>(); 
    image_ = (uint8_t*) frame.request().ptr; 
    calc_region_hist(roi_dist_, (uint32_t)x_val, (uint32_t)y_val, (uint32_t)w, (uint32_t)h);  
    image_ = nullptr; 

    return_state_ = py::array_t<double>({particle_num_});
}

// adding 0-mean Gaussian Noise
inline void FaceTrackerPF::control_callback(std::vector<double>& states){
  //TODO: not gaussian noise
  std::vector<double> std_devs; 
  std_devs.reserve(states.size()); 
  for (unsigned int i = 0; i < states.size(); ++i){
    std_devs.emplace_back(Filter::Util::generate_random_num_universal(0.0, 1.0, 1)[0]);
  }
  states.at(X) += (states.at(VX)*delta_t_ + std_devs.at(0) * states.at(HX));     //TODO: add noise
  states.at(Y) += (states.at(VY)*delta_t_ + std_devs.at(1) * states.at(HY));     //TODO: add noise
  states.at(VX) += (std_devs.at(2)*velocity_disturb_); 
  states.at(VY) += (std_devs.at(3)*velocity_disturb_); 
  states.at(HX) += (std_devs.at(4)*scale_change_disturb_ + states.at(AT_DOT) * delta_t_); 
  states.at(HY) += (std_devs.at(5)*scale_change_disturb_ + states.at(AT_DOT) * delta_t_); 
  states.at(AT_DOT) += (std_devs.at(6)*scale_change_disturb_); 
}

//TODO - to compare
inline double FaceTrackerPF::observation_callback(const std::vector<double>& state_estimate){
    std::vector<double> hist(512, 0);  
    // calculate the histogram of the state. 
    calc_region_hist(hist, state_estimate.at(X), state_estimate.at(Y), state_estimate.at(HX), state_estimate.at(HY)); 
    // calculate the Bhattacharya Coefficient between the ROI histogram and the state's histogram 
    // To check: sum up all weights, then normalize

    return 0; 
}

// Store our custom pixel value in image_ into the histogram. custom pixel value = k_pixel * raw_pixel_value is in [0,512], and each value is composed of: [3bits_for_B | 3bits_for_G | 3bits_for_R]. K is the kernal function value. (x,y) is the center, (w, h) are the width and height of a region
inline void FaceTrackerPF::calc_region_hist (std::vector<double>& hist, uint32_t x_center, uint32_t y_center, uint32_t w, uint32_t h){

  auto dist = [&x_center , &y_center](double x, double y){
      double x_diff = x - (double)x_center; 
      double y_diff = y - (double)y_center;
      return std::sqrt(x_diff * x_diff + y_diff * y_diff);
  };
  double half_diag = std::sqrt(w*w + h*h)/2.0; 
  uint32_t x_begin = std::max((uint32_t) 0, x_center - w/2);
  uint32_t y_begin = std::max((uint32_t) 0, y_center - h/2);
  uint32_t x_end = std::min((uint32_t)ranges_vec_.at(X).second - 1, x_center + w/2);
  uint32_t y_end = std::min((uint32_t)ranges_vec_.at(Y).second - 1, y_center + h/2);
  // initialize histogram to 0; 
  hist = std::vector<double>(512, 0.0); 
  double sum_k = 0.0; 
  for (auto x = x_begin; x<x_end; ++x)
    for (auto y = y_begin; y < y_end; ++y){
      double r = dist(x, y) / half_diag;
      double k = 1 - r*r; 
      sum_k += k; 
      uint32_t width = (uint32_t)ranges_vec_.at(X).second; 

      uint8_t blue = image_[ 3 * (width - 1) * y + x ] >> RIGHT_SHIFT; 
      uint8_t green = image_[ 3 * (width - 1) * y + x + 1 ] >> RIGHT_SHIFT; 
      uint8_t red = image_[ 3 * (width - 1) * y + x + 2 ] >> RIGHT_SHIFT; 
      uint8_t bin_index = blue << (2 * RIGHT_SHIFT) | green << RIGHT_SHIFT | red; 

      hist.at(bin_index) += k; 
    }

  // normalize
  std::for_each(hist.begin(),hist.end(), [&sum_k](double& num){num /= sum_k;});
}

/*
 * Particle Filter Workflow: 
 * 1. Input frame in RGB, store the frame reference
 * - Filter::Particle_Filter::run() resamples states based on weights
 * 2. Filter::Particle_Filter::run() will call control_callback to predict states. 
 * 3. With updated state predictions, Filter::Particle_Filter::run() calls observation_callback()
 * - Return all states in double
 */
inline py::array_t<double> FaceTrackerPF::run_one_iteration(const py::array_t<double>& frame){
   // particle_filter will launch a thread pool that calls the callbacks
   image_ = (uint8_t*) frame.request().ptr; 
   std::vector<double> belief = pf_ -> run(); 
   adjust_belief(belief); 
   memcpy((double*)return_state_.request().ptr, belief.data(), sizeof(double) * belief.size()); 
   image_ = nullptr;
   return return_state_; 
}

// Making the estimated rectangle fall inside the frame.
inline void FaceTrackerPF::adjust_belief(std::vector<double>& belief){
     auto& x_ref = belief.at(X); 
     x_ref = (ranges_vec_.at(X).first < x_ref ) ? x_ref : ranges_vec_.at(X).first; 
     x_ref = (x_ref < ranges_vec_.at(X).second) ? x_ref : ranges_vec_.at(X).second -1; 
     auto& y_ref = belief.at(Y);
     y_ref = (ranges_vec_.at(Y).first < y_ref ) ? y_ref : ranges_vec_.at(Y).first;
     y_ref = (y_ref < ranges_vec_.at(Y).second) ? y_ref : ranges_vec_.at(Y).second -1;
}
#pragma GCC visibility pop
