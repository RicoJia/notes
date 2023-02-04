#! /usr/bin/env python

import unittest
import rospy
import rostest
from logging import getLogger
from test_pkg.msg import StringList


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
        private_value = rospy.get_param('~private_value', None)
        rospy.logwarn("==============rospy loginfo")
        getLogger("python logger").warn("********rospy logger")
        print("value: ", value, ", private_value: ", private_value)
        self.assertIsNotNone(value)

class TestPubSub(unittest.TestCase):
    def setUp(self):
        """
        Quirks:
            - Setup is only run when there's a test, and at the beginning of every test
        """
        self.sub = rospy.Subscriber("rico_topic", StringList, self.sub_cb)
    def tearDown(self):
        # Have to unregister because subscriber won't be destroyed in tearDown
        self.sub.unregister()
    def sub_cb(self, msg):
        print(msg)

if __name__ == '__main__':
    rospy.init_node('test_params')
    rostest.rosrun('tutorial', 'test_params', MyTestCase)
    rostest.rosrun('tutorial', 'test_params', TestPubSub)
