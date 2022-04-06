#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <sophus/so2.hpp>

// Heads-up: linkage order is important, in this case you this file needs symbols from the fmt library, so do g++  sophus_example.cpp -lfmt, not g++ -lfmt sophus_example.cpp. 

void test_hat(){
    auto delta_R = Sophus::SO2d::exp(5); 
    delta_R = Sophus::SO2<double>::exp(5); 
}

int main()
{
    test_hat();
}
