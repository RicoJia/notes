#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <vector>
#include <cstring> //memcpy
#include <string>
#include <exception>
#include <thread>
#include <map>
#include <iostream>
#include <chrono>
#include <cstdio>
#include <functional>

using namespace std::chrono_literals; 

using namespace std;
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

void single_threaded_test(PickleConverter& pc, const std::string& file_name){
  // make sure each thread has one PickleConverter instance, so there's no potential race condition. 
  auto tup = py::make_tuple("heng", file_name);
  pc.open(file_name);
  pc.write_pickle(tup); 
  pc.close(); 
}

using PickleConverterCallback = py::function;
void pybind_test(){
  PickleConverter pc; // Note python modules are singletons, GIL must be required. 
  single_threaded_test(pc, "hee.txt"); 
}


PYBIND11_MODULE(example, m) { //module name must match file name
    // optional module docstring
    m.doc() = "pybind11 example plugin";
    // define add function
    m.def("pybind_test", &pybind_test, "A function which adds two numbers");
}

