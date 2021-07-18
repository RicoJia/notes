#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <vector>
#include <cstring> //memcpy
#include <exception>

#include <map>
#include <iostream>

using namespace std;
namespace py = pybind11;

void load_map_python_dict(const py::dict& python_dict, std::map<std::string, float> &outMap) {
    for(const std::pair<py::handle, py::handle>& item: python_dict){
        auto key = item.first.cast<std::string>(); 
        auto value = item.second.cast<float>();
        outMap[key] = value; 
    }
}

void pybind_test(const py::list& ls){
      cout<<ls.size()<<endl;
      for (const auto& item : ls){
          auto sub_list  = item.cast<py::list>();
          for (const auto& it : sub_list){
            cout<<it.cast<int>()<<endl;
          }
      }
}


PYBIND11_MODULE(example, m) { //module name must match file name
    // optional module docstring
    m.doc() = "pybind11 example plugin";
    // define add function
    m.def("pybind_test", &pybind_test, "A function which adds two numbers");

}

