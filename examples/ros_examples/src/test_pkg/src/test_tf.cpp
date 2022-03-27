#include <iostream>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include "tf/message_filter.h"

using std::cout; using std::endl; 
/**
 * Notes:
 *  q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;       
    2. For plannar robot
    theta = 2*atan(q.z, q.w); //z = sy, w = cy, where sy = sin(0.5yaw)
*/
void quat_calc()
{
    tf2::Quaternion q; 
}

/**
* @brief: This can be used in ROS. https://stackoverflow.com/questions/33117457/conversions-between-tf-and-eigen
* Great Tutorial: https://github.com/felipepolido/EigenExamples
* 1. Affine3f is defied as: 
*  linear translation
*  0...0  1
* 2. use Affine3f::matrix() for printing
*/
void test_affine()
{
    Eigen::Affine3f transform_temp = Eigen::Affine3f::Identity(); 
    transform_temp.translation() = Eigen::Vector3f(1.0, 1.0, 2.0); 
    transform_temp.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f(0,0,1)));

    cout<<transform_temp.matrix()<<endl;
    // transform_temp.linear() = 

    // const Eigen::Matrix3d rotation_part = transform_temp.rotation().cast<double>();
    // const Eigen::Vector3d translation_part = transform_temp.translation().cast<double>();
    // tf::Transform transform_tf;
    // tf::transformEigenToTF(transform_eigen, transform_tf);
}

/**
* @ Common Commands
*   - rosrun tf tf_echo /map /base_scan
*   - rosrun tf view_frames
*   - rosrun tf tf_monitor
*/
int main(int argc, char**argv)
{
    ros::init(argc, argv, "test_tf"); 
    test_affine();
    
}
