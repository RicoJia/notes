#include <iostream>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
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
* @brief: TF2 Theory
*   1. TF2 is faster: static_transform_publisher (using latched topic)
*       rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 /base_link /laser, publishing on /tf_static
*   2. TF2 坐标变换监听器中的 Buffer 实现 is faster too 
    *   - tf::buffer: cache time - will keep tf for cached time, 10s
    *   - The cached time is implemented as a linked list
*   3. tf_kdl ... are instances of the templated class tf2. You can define your own, too.
*   4. Action-based client for occasional queries for transform (instead of having a listener constntly listening)
*/
void test_tf2(){
    geometry_msgs::TransformStamped transformStamped;
    // without specifying cached time, it's 10s by default
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    try{
        // note this ros::Time(0) returns the "latest message"
      transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
      // Note: we have now, but buffer time is 3s. We can't have now without 3s because that way, the transform will be invalid.
      transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time::now(), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
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
