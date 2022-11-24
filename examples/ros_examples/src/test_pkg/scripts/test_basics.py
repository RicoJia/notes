#!/usr/bin/env python
"""
1. Note that shbang should not have space in the middle
2. Timer callback
"""

import rospy

def test_timer_cb(timer_event: rospy.timer.TimerEvent):
    print("timer cb", timer_event.__dict__)

if __name__ == "__main__":
    rospy.init_node("test_basics") 
    rospy.loginfo("hello")
    timer = rospy.Timer(period=rospy.Duration.from_sec(1), callback=test_timer_cb)
    rospy.spin()

