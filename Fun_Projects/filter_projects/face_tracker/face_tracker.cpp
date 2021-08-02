#include "face_tracker.hpp"

// #ifdef DEBUG
// class FaceTrackerPFDebug : public FaceTrackerPF{
//     public: 
//       using FaceTrackerPF::adjust_belief; 
// }; 
// PYBIND11_MODULE(face_tracker, m) { //module name must match file name
//     // optional module docstring
//     m.doc() = "A color based  object tracker using particle filter";
//     py::class_<FaceTrackerPFDebug>(m, "face_tracker_pf_debug")
//       .def(py::init<const py::dict&>(), py::arg("inputs"))
//       .def("run_one_iteration", &FaceTrackerPFDebug::run_one_iteration);
//       .def("adjust_belief", &FaceTrackerPFDebug::adjust_belief);
// }
// #else
PYBIND11_MODULE(face_tracker, m) { //module name must match file name
    // optional module docstring
    m.doc() = "A color based  object tracker using particle filter";
    py::class_<FaceTrackerPF>(m, "face_tracker_pf")
      .def(py::init<const py::dict&>(), py::arg("inputs"))
      .def("run_one_iteration", &FaceTrackerPF::run_one_iteration);
}
// #endif

