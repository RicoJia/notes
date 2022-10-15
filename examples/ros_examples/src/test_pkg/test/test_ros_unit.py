#! /usr/bin/env python

import unittest
import rospy
import rostest
from logging import getLogger

# 1. rostest args: http://wiki.ros.org/roslaunch/XML/test No output
# 2. manually run rostest: http://wiki.ros.org/rostest/Commandline

class MyTestCase(unittest.TestCase):
    # comment this out if you don't need to start ros topics, etc.
    # def __init__(self, *args, **kwargs):
    #     rospy.init_node('test_params')

    def test_param_loaded(self):
        value = rospy.get_param('/value', None)
        rospy.logwarn("==============rospy loginfo")
        getLogger("python logger").warn("********rospy logger")
        print("********hello")
        self.assertIsNotNone(value)


if __name__ == '__main__':
    
    rostest.rosrun('tutorial', 'test_params', MyTestCase)
