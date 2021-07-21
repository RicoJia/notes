#include <pybind11/pybind11.h>
#include <memory>
#include "particle_filter.hpp"

namespace py = pybind11;

using Filter::Particle_Filter; 
class Face_tracker_PF{
  public: 
    explicit Face_tracker_PF(const py::list& ranges, const int& particle_num){
        std::vector<std::pair<double, double>> ranges_vec; 
        for(const auto& item : ranges){
          auto range = item.cast<py::list>();
          ranges_vec.emplace_back(std::make_pair<double, double>(range[0].cast<double>(), range[1].cast<double>())); 
        }
        // pf_ = std::make_unique<Particle_Filter>(ranges_vec, particle_num); 
    }
  private: 
    std::unique_ptr<Particle_Filter> pf_; 


}; 



PYBIND11_MODULE(face_tracker, m) { //module name must match file name
    // optional module docstring
    m.doc() = "A color based  object tracker using particle filter";
    py::class_<Face_tracker_PF>(m, "face_tracker_pf")
      .def(py::init<const py::list&, const int&>(), 
                                  py::arg("ranges"),
                                  py::arg("particle_num") = 100);
}

