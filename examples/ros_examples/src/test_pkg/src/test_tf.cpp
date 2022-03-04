#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

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

int main(int argc, char**argv)
{
    ros::init(argc, argv, "test_tf"); 
    
}
