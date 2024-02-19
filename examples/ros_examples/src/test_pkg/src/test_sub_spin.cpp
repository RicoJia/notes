#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>
using namespace std;

int shared_resource = 0; // Example shared resource

void callback1(const std_msgs::String::ConstPtr &msg) {
  // Access shared_resource here without mutex since we're using ros::spin()
  shared_resource++;
  cout << "cb1: shared_resource: " << shared_resource << endl;
  sleep(1);
  cout << "cb1: shared_resource after sleep: " << shared_resource << endl;
}

void callback2(const std_msgs::String::ConstPtr &msg) {
  // Access shared_resource here without mutex since we're using ros::spin()
  shared_resource--;
  cout << "cb2: shared_resource: " << shared_resource << endl;
}

/**
 * Conclusion: ros::spin() is truly threadsafe. Running this node I see
    cb1: shared_resource: 1
    cb1: shared_resource after sleep: 1
    cb2: shared_resource: 0
    ...
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "example_node");
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe("topic1", 10, callback1);
  ros::Subscriber sub2 = nh.subscribe("topic2", 10, callback2);
  ros::Publisher pub1 = nh.advertise<std_msgs::String>("topic1", 1000);
  ros::Publisher pub2 = nh.advertise<std_msgs::String>("topic2", 1000);
  while (ros::ok()) {
    std_msgs::String msg;
    msg.data = "hello";
    pub1.publish(msg);
    pub2.publish(msg);
    ros::spinOnce(); // Sequentially processes callbacks in a single thread
    sleep(2);
  }

  return 0;
}
