// TO run this example, do g++ -I /usr/include/eigen3/ eigen_example.cpp
// #include <Eigen/src/Geometry/Transform.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;

void test_creation(){
    Matrix3f A;

    Matrix4d B;
    B = Matrix4d :: Identity();
    B = Matrix4d :: Zero();
    B = Matrix4d :: Ones();
    B = Matrix4d :: Constant (4.5) ;

    A << 1.0, 2.0, 3.0, 
     1.0, 2.0, 3.0,
     1.0, 2.0, 3.0;
}

void test_auto_keyword(){
    // DO NOT USE auto, initialize matrices properly
    // auto R_3d = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_3d = Eigen::Matrix3d::Identity();
    R_3d.block<2,2>(0,0) = Eigen::Matrix2d::Identity();
}
 
void test_block_and_larger_shape(){
    // if d more than 4, do ```Eigen::Matrix<double, 6, 6>```
    // Also, need to initialize it with Zero
    Eigen::Matrix<double, 6, 6> mat = Eigen::Matrix<double, 6, 6>::Zero();
    // this is how we initialize a block
    mat.block<2,2>(0,0) = Eigen::Matrix2d::Identity();
    cout<<"test block: "<<mat<<endl;

    // Elements with **length up to 4 can do** this:
    Vector3f w(1.0f, 2.0f, 3.0f);

    auto i = w.tail<2>();
    cout<<"i: "<<i<<endl;

    std::cout<<__FUNCTION__<<": element: "<<w(2)<<std::endl;
}

void test_vector_min(){
    // MatrixXd m = MatrixXd::Random(3,3);
    // m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
    // cout << "m =" << endl << m << endl;
    VectorXd v(3);
    v << 1, 2, 3;

    Vector3d v2 = {-2.0, 3.0, 4.0}; 
    // cout<<v2.array().abs().minCoeff();

    auto v3 = VectorXd::Zero(3);
    cout<<v3; 
}

void test_affine(){
    Eigen::Affine3d af;
    cout<<af.matrix()<<endl;  // will see all zeros
    Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(1, 0, 0)));
    cout<<rx.matrix()<<endl;  // will see all zeros
    Eigen::Affine3d af2 = Eigen::Affine3d::Identity();
    cout<<af2.matrix()<<endl;  // will see all zeros
}

int main()
{
    test_creation(); 
    test_block_and_larger_shape();
}
