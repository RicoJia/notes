#!/usr/bin/env python
"""
1. Note that shbang should not have space in the middle
2. Timer callback
"""

import rospy
from test_pkg.msg import StringList
import json
import numpy as np

def test_pub_sub_connection():
    pub = rospy.Publisher( 
                          "rico_topic",
                          StringList,
                          # need queue_size to suppress warning
                          queue_size = 3
    )
    # here we must have a delay so the publisher is built completely
    rospy.sleep(1)

    # Alternatively, we can use get_num_connections() to wait for a subscriber
    
    print_interval = 1
    def check_connections():
        try:
            t0 = rospy.core.time.time()
            t1 = t0 + print_interval
            while not rospy.is_shutdown() and rospy.core.time.time() < t1:
                if pub.get_num_connections() > 0:
                    print("publisher-subscriber are fine")
                    return True
                # TODO: exit sleep early if connection is established
                rospy.core.time.sleep(0.1)
        except rospy.ROSException as e:
            return False

    check_connections()
    pub.publish(StringList(["sdf", "dfsa"]))

def test_subscriber():
    """
    dummy example that shows: initialize subscribers last SO VARIABLES CAN FINISH INITIALIZING!
    """
    class SomeClass:
        def __init__(self):
            self.var = 1
            self.sub = rospy.Subscriber("/test_topic", StringList, self.sub)
        def sub_cb(self, msg):
            pass

def test_timer_cb(timer_event: rospy.timer.TimerEvent):
    print("timer cb", timer_event.__dict__)

def test_rosrun_args():
    """
    rosrun test_pkg test_basics _test:="test" 
    """
    t = rospy.get_param("~test")
    print("t: ", t)

    # another example: rosparam.get_param("Field1/Field2")
    # In Yaml, it can be: 
    # - Field 1
    #   - Field 2


    # Another thing to note, is after running launch file 
    # that loads params, The params can still exist

def test_env_var():
    import os
    print(f"Testing OS env var: {os.getenv('test_env_var')}")
    print(f"Testing private OS env var: {os.getenv('private_test_env_var')}")
    print(f"rosparam, optenv private_opt_env: {rospy.get_param('~private_opt_env')}")
    print(f"rosparam, optenv opt_env: {rospy.get_param('opt_env')}")

if __name__ == "__main__":
    rospy.init_node("test_basics") 
    rospy.loginfo("hello")
    # timer = rospy.Timer(period=rospy.Duration.from_sec(1), callback=test_timer_cb)
    test_pub_sub_connection()
    test_env_var()

    rospy.spin()

