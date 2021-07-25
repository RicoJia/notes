#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <memory>
#include <vector>
#include "particle_filter.hpp"

#pragma GCC visibility push(hidden)
namespace py = pybind11;

using Filter::Particle_Filter; 
//TODO
using std::cout; using std::endl; 

enum State_Index{
  x = 0, 
  y = 1, 
  vx = 2, 
  vy = 3,
  hx = 4,
  hy = 5,
  at_dot = 6
}; 

class Face_tracker_PF{
  public: 
    explicit Face_tracker_PF(const py::dict& inputs); 
    py::array_t<double> run_one_iteration(const py::array_t<double>& frame);
  private: 
    std::unique_ptr<Particle_Filter> pf_; 
    // pre-allocating return state so it's fast
    py::array_t<double> return_state_;    
    
    //takes in a set of states, and apply control with noise. 
    void control_callback(std::vector<double>& states);
    // Given state estimate, return the likelihood of the current observation
    double observation_callback(const std::vector<double>& state_estimate);

    void adjust_belief(std::vector<double>& belief); 

    std::vector<std::pair<double, double>> ranges_vec_; 
    std::vector<double> noise_vec_; 
    const int particle_num_; 
    const double scale_change_disturb_; 
    const double velocity_disturb_; 
    const double delta_t_; 
}; 

Face_tracker_PF::Face_tracker_PF(const py::dict& inputs) : 
    particle_num_(inputs["PARTICLE_NUM"].cast<int>()), 
    scale_change_disturb_(inputs["SCALE_CHANGE_DISTURB"].cast<double>()),
    velocity_disturb_(inputs["VELOCITY_DISTURB"].cast<double>()), 
    delta_t_(1.0/inputs["FRAME_RATE"].cast<float>()), 
    return_state_(py::array_t<double>({particle_num_}))
{
    // ranges is [(upper_lim, lower_lim, standard_deviation_of_noise), ...]
    auto ranges = inputs["ranges"].cast<py::list>(); 
    for(const auto& item : ranges){
      auto range = item.cast<py::list>();
      ranges_vec_.emplace_back(std::make_pair<double, double>(range[0].cast<double>(), range[1].cast<double>())); 
      noise_vec_.emplace_back(range[2].cast<double>());   
    }
    pf_ = std::make_unique<Particle_Filter>(ranges_vec_, particle_num_); 

    pf_ -> register_control_callback(std::bind(&Face_tracker_PF::control_callback, this, std::placeholders::_1)); 
    pf_ -> register_observation_callback(std::bind(&Face_tracker_PF::observation_callback, this, std::placeholders::_1));
}

// adding 0-mean Gaussian Noise
void Face_tracker_PF::control_callback(std::vector<double>& states){
  //TODO: not gaussian noise
  auto std_devs = Filter::Particle_Filter::generate_random_num_universal(0.0, 1.0, states.size());
  states.at(x) += (states.at(vx)*delta_t_ + std_devs.at(0) * states.at(hx));     //TODO: add noise
  states.at(y) += (states.at(vy)*delta_t_ + std_devs.at(1) * states.at(hy));     //TODO: add noise
  states.at(vx) += (std_devs.at(2)*velocity_disturb_); 
  states.at(vy) += (std_devs.at(3)*velocity_disturb_); 
  states.at(hx) += (std_devs.at(4)*scale_change_disturb_ + states.at(at_dot) * delta_t_); 
  states.at(hy) += (std_devs.at(5)*scale_change_disturb_ + states.at(at_dot) * delta_t_); 
  states.at(at_dot) += (std_devs.at(6)*scale_change_disturb_); 
}

double Face_tracker_PF::observation_callback(const std::vector<double>& state_estimate){
    return 0; 
}

/*
 * Particle Filter Workflow: 
 * 1. Input frame in RGB, store the frame reference
 * - Filter::Particle_Filter::run() resamples states based on weights
 * 2. Filter::Particle_Filter::run() will call control_callback to predict states. 
 * 3. With updated state predictions, Filter::Particle_Filter::run() calls observation_callback()
 * - Return all states in double
 */
py::array_t<double> Face_tracker_PF::run_one_iteration(const py::array_t<double>& frame){
   // particle_filter will launch a thread pool that calls the callbacks
   std::vector<double> belief = pf_ -> run(); 
   adjust_belief(belief); 
   memcpy((double*)return_state_.request().ptr, belief.data(), sizeof(double) * belief.size()); 
   return return_state_; 
}

// Making the estimated rectangle fall inside the frame.
void Face_tracker_PF::adjust_belief(std::vector<double>& belief){
     auto& x_ref = belief.at(x); 
     x_ref = (ranges_vec_.at(x).first < x_ref ) ? x_ref : ranges_vec_.at(x).first; 
     x_ref = (x_ref < ranges_vec_.at(x).second) ? x_ref : ranges_vec_.at(x).second -1; 

     auto& y_ref = belief.at(y);
     y_ref = (ranges_vec_.at(y).first < y_ref ) ? y_ref : ranges_vec_.at(y).first;
     y_ref = (y_ref < ranges_vec_.at(y).second) ? y_ref : ranges_vec_.at(y).second -1;
}

#pragma GCC visibility pop

PYBIND11_MODULE(face_tracker, m) { //module name must match file name
    // optional module docstring
    m.doc() = "A color based  object tracker using particle filter";
    py::class_<Face_tracker_PF>(m, "face_tracker_pf")
      .def(py::init<const py::dict&>(), py::arg("inputs"))
      .def("run_one_iteration", &Face_tracker_PF::run_one_iteration);
}

