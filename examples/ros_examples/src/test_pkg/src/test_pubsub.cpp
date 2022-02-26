#include <ros/ros.h>

int main(int argc, char**argv)
{
    ros::init(argc, argv, "Hello"); 

    // establish this as a ros node
    ros::NodeHandle nh; 

    ROS_INFO_STREAM("hola"); 
}
