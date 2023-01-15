#! /usr/bin/env python

import unittest
import rospy
import rostest
from logging import getLogger

# Do not directly run this test. Run the one in launch instead
# 1. rostest args: http://wiki.ros.org/roslaunch/XML/test No output.
# 2. manually run rostest: http://wiki.ros.org/rostest/Commandline

class MyTestCase(unittest.TestCase):
    # comment this out if you don't need to start ros topics, etc.
    # def __init__(self, *args, **kwargs):
    #     rospy.init_node('test_params')

    def test_param_loaded(self):
        # since we load this parameter in .test, we can't pass args from command line
        value = rospy.get_param('value1', None)
        rospy.logwarn("==============rospy loginfo")
        getLogger("python logger").warn("********rospy logger")
        print("value: ", value)
        self.assertIsNotNone(value)


if __name__ == '__main__':
    
    rostest.rosrun('tutorial', 'test_params', MyTestCase)
