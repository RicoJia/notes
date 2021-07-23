#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <memory>
#include "particle_filter.hpp"

#pragma GCC visibility push(hidden)
namespace py = pybind11;

using Filter::Particle_Filter; 
//TODO
using std::cout; using std::endl; 


class Face_tracker_PF{
  public: 
    explicit Face_tracker_PF(const py::list& ranges, const int& particle_num); 
    py::array_t<double> run_one_iteration(const py::array_t<double>& frame);
  private: 
    std::unique_ptr<Particle_Filter> pf_; 
    // pre-allocating return state so it's fast
    py::array_t<double> return_state_;    
    
    //takes in a set of states, and apply control with noise. 
    void control_callback(std::vector<double>& states);
    // Given state estimate, return the likelihood of the current observation
    double observation_callback(const std::vector<double>& state_estimate);

}; 

Face_tracker_PF::Face_tracker_PF(const py::list& ranges, const int& particle_num) : 
    return_state_(py::array_t<double>({particle_num}))
{
    std::vector<std::pair<double, double>> ranges_vec; 
    for(const auto& item : ranges){
      auto range = item.cast<py::list>();
      ranges_vec.emplace_back(std::make_pair<double, double>(range[0].cast<double>(), range[1].cast<double>())); 
      cout<<ranges_vec.back().first<<" , "<<ranges_vec.back().second<<endl;
    }
    pf_ = std::make_unique<Particle_Filter>(ranges_vec, particle_num); 
    // register callbacks
    pf_ -> register_control_callback(std::bind(&Face_tracker_PF::control_callback, this, std::placeholders::_1)); 
    pf_ -> register_observation_callback(std::bind(&Face_tracker_PF::observation_callback, this, std::placeholders::_1));
}

void Face_tracker_PF::control_callback(std::vector<double>& states){
    //TODO
    cout<<__FUNCTION__<<" | "<<states.size()<<endl;
}

double Face_tracker_PF::observation_callback(const std::vector<double>& state_estimate){
    //TODO
    return 0; 
}

py::array_t<double> Face_tracker_PF::run_one_iteration(const py::array_t<double>& frame){
   // particle_filter will launch a thread pool that calls the callbacks
   std::vector<double> belief = pf_ -> run(); 
   memcpy((double*)return_state_.request().ptr, belief.data(), sizeof(double) * belief.size()); 
   return return_state_; 
}

#pragma GCC visibility pop

PYBIND11_MODULE(face_tracker, m) { //module name must match file name
    // optional module docstring
    m.doc() = "A color based  object tracker using particle filter";
    py::class_<Face_tracker_PF>(m, "face_tracker_pf")
      .def(py::init<const py::list&, const int&>(), 
                                  py::arg("ranges"),
                                  py::arg("particle_num") = 100)
      .def("run_one_iteration", &Face_tracker_PF::run_one_iteration);
}

