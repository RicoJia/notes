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
    """
    General Test Cases:
        - test cases are run sequentially, but its sequence is random
        - -t / --text
        - rostest PKG test_file --time_limit, by default it's 60s
        6. When the test is done, rostest will call all deleters. 
        7. When it's difficult to debug, make the test a "non test object"
    """
    def setUp(self):
        """
        Quirks:
            -  setUp and tearDown are run per test case
            - if in setUp we set a rostopic sub, we must have teardown to close it out
        """
        self.sub = rospy.Subscriber("rico_topic", StringList, self.sub_cb)
    def tearDown(self):
        # Have to unregister because subscriber won't be destroyed in tearDown
        self.sub.unregister()
    def sub_cb(self, msg):
        print(msg)
    def test_dummy(self):
        print("dummy test")

if __name__ == '__main__':
    rospy.init_node('test_params')
    rostest.unitrun('tutorial', 'test_params', MyTestCase)
    rostest.unitrun('tutorial', 'test_params', TestPubSub)
