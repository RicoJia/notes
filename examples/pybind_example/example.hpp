#ifndef __EXAMPLE_HPP__
#define __EXAMPLE_HPP__

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <vector>
#include <cstring> //memcpy
#include <string>
#include <exception>
#include <thread>
#include <map>
#include <iostream>
#include <cstdio>
#include <functional>

// #pragma GCC visibility push(hidden)
namespace py = pybind11;
using namespace py::literals;

class PickleConverter{
    public: 
      PickleConverter() : pickle_(py::module::import("pickle")){
      }
      void open(const std::string& file_name){
          file_name_ = file_name;
      }
      void close() {}
      void write_pickle(const py::object& obj){
          auto picked_bytes = pickle_.attr("dumps")(obj, 0).cast<py::bytes>();

          // TODO replace
          std::string picked_bytes_str = picked_bytes.cast<std::string>();    //only way to get usable pickled_bytes 
          FILE* file = fopen(file_name_.c_str(), "wb"); 
          fwrite(picked_bytes_str.c_str(), sizeof(char), picked_bytes_str.size(), file);
          fclose(file); 
        }
    private: 
      // pybind11::module pickle_ = py::module::import("pickle");; 
      pybind11::module pickle_; 
      std::string file_name_; 
};

void foo(); 
// #pragma GCC visibility pop

#endif /* end of include guard: __EXAMPLE_HPP__ */
