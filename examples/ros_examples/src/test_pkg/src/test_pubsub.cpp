#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <functional>

/**
 * Notes: 
 *  1. ROS_DEBUG_THROTTLE uses polling. 
 *      - Polling is like sleep() with higher resolution: the host constantly checks if the controller is ready for I/O. (when the controller is ready, the comunication will start, otherwise, it always checks). It's called synchronous as the caller needs to wait
 *      - Interrupts are: generate a short service routine, execute it, and return the control back. It's also called "asynchronous", because after initiating the call, the caller can go back and do something else
 *  2. DEBUG, INFO are sent to std_out, WARN, ERROR, FATAL are sent to std_err(both are streams, and you can direct them to a file thru pipes.)
 *  3. by default, default msgs won't show. Try 
 *      1. rosrun rqt_logger_level rqt_logger_level, select node
 *      2. like above
 *      3. node/set_logger_level service, like rosservice call /Hello/set_logger_level  "logger: 'ros.test_pkg' level: 'info'". Check roslogger
 *      4. Launch file: <node pkg="rosservice" type="rosservice" name="set_planner_interface_aw_log_level" args="call --wait /planner_interface_aw/set_logger_level 'ros.aw_navigation' 'fatal'" />
 */
void logging_and_sleep()
{
    ROS_INFO_STREAM("hola");
    // Don't see anything on the console tho
    ROS_INFO_NAMED("test_only", "Hello %s", "World");
    // supports c++ style stream
    ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": sub - ");
    int x = 0;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        // Must be called, else some previously hit logging statements? may still / still not print
        ros::console::notifyLoggerLevelsChanged();
    }
    ROS_DEBUG_COND(x==0, "cond true");
    for (unsigned int i = 0; i < 10; ++i) {
        ROS_DEBUG_THROTTLE(1, "This prints every 1s max");
        ROS_DEBUG_ONCE("This message will only print once");
        ros::Duration(0.5).sleep();
    }
}

/**
 * Notes: 
 *  1. Every message type starts with cap letter <geometry_msgs/Twist.h>. Twist is defined in the namespace geometry_msgs
 *  2. ROS client library (the one contains pub/sub) knows which subs and pubs belong to the same underlying process. In this case, the msgs can be transported directly from publisher to subscriber. Same thing is used for nodelets
 *  3. create ros::Publisher is expensive
 *  4. Dead sinks = topic with no publisher. Leaf topics: topic with no subscriber. debug: All topics used for debugging. 
*/
void pub(ros::NodeHandle& nh)
{
    // Note with /cmd_vel it will be absolute naming. Else it's relative. 50 is the queue size
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 30; 
    twist_msg.angular.x = 30; 
    ros::Rate r(2); 
    for (unsigned int i = 0; i < 50; ++i) {
        // use ctrl-c, or rosnode kill
        if (ros::ok()){
            vel_pub.publish(twist_msg);
            r.sleep();
        }
    }
}

// Subscriber callback, note we have the constptr
void cb1(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": sub - "<<msg->linear.x); 
}

// Subscriber callback, note we have the constptr
void cb2(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": sub - "<<msg->linear.x); 
}
/**
 * Notes: 
 *  1. I tried lambda, do not work here!
 *  2. ros::spinOnce() is for single threaded. Therefore, the subscribers work single-threaded model
*/
void sub_bind_spin(ros::NodeHandle& nh)
{
    ros::Subscriber sub1 = nh.subscribe<geometry_msgs::Twist>("/sub_cmd_vel", 1000, cb1); 
    ros::Subscriber sub2 = nh.subscribe<geometry_msgs::Twist>("/sub_cmd_vel", 1000, cb2); 
    // std::bind no problem here
    auto dummy_func = std::bind(cb1, nullptr);

    // have 2 threads processing callbacks, a thread pool servicing callbacks
    // ros::MultiThreadedSpinner spinner(2); 
    // spinner.spin(); 

    // More useful since you can start the spinner later
    // ros::AsyncSpinner spinner(4); // Use 4 threads
    // spinner.start();
    // ros::waitForShutdown();
}

/**
 * Notes: 
 *  1. rosparam set var_name value 
 *  2. rosparam get parameter_name
 *  3. rosparam delete parameter_name 
 *  4. set params 
 *      
*/
void set_get_param(ros::NodeHandle& nh){
    std::string test_var, global_test_var, default_test_var;
    // 1. use nh
    nh.getParam("test_var", test_var); 
    ROS_INFO_STREAM("get test_var: "<<test_var);
    nh.getParam("/global_test_var", global_test_var); 
    ROS_INFO_STREAM("get global test_var: "<<global_test_var);
    nh.param<std::string>("default_test_var", default_test_var, "default string"); 
    ROS_INFO_STREAM("default_test_var: "<<default_test_var);

    // 2. use baremetal, but names are still relative to node's namespace
    ros::param::get("test_var", test_var); 
    ROS_INFO_STREAM("get test_var baremetal: "<<test_var);

    // 3. checks - namespace is relative here
    if (nh.hasParam("test_var")) ROS_INFO_STREAM("Yeehaw"); 
}

int main(int argc, char**argv)
{
    /**
    * @brief: ROS Node 
    */
    // node name is hello
    ros::init(argc, argv, "Hello"); 
    // establish this as a ros node
    // 1. will make everything (topic, etc) global_namespace/topic
    // ros::NodeHandle nh("global_namespace"); 
    // 2. will make everything (topic, etc) <node_name>/private_namespace/topic
    // ros::NodeHandle nh("~private_namespace"); 
    // 3. will make everything (topic, etc) <node_name>/topic
    ros::NodeHandle nh("~"); 

    // logging_and_sleep();
    // pub(nh); 
    // sub_bind_spin(nh); 
    //
    set_get_param(nh);


    // shutdown the node
    // ros::shutdown();

}
