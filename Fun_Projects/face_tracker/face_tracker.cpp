#include <pybind11/pybind11.h>
#include "particle_filter.hpp"

namespace py = pybind11;

void initialize(){

}

PYBIND11_MODULE(face_tracker, m) { //module name must match file name
    // optional module docstring
    m.doc() = "A color based  object tracker using particle filter";
    // define add function
    m.def("initialize", &initialize); 

}

