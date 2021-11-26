// TO run this example, do g++ -I /usr/include/eigen3/ eigen_example.cpp
#include <iostream>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;
 
int main()
{
  // MatrixXd m = MatrixXd::Random(3,3);
  // m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
  // cout << "m =" << endl << m << endl;
  VectorXd v(3);
  v << 1, 2, 3;

  Vector3d v2 = {-2.0, 3.0, 4.0}; 
  // cout<<v2.array().abs().minCoeff();

  auto v3 = VectorXd::Zero(3);
  v3(1) = 0;
  cout<<v3; 
}
