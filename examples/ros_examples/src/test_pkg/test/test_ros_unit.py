#! /usr/bin/env python

import unittest
import rospy
import rostest
from logging import getLogger

class MyTestCase(unittest.TestCase):
    def test_param_loaded(self):
        value = rospy.get_param('/value', None)
        rospy.logwarn("==============rospy loginfo")
        getLogger("python logger").warn("********rospy logger")
        print("********hello")
        self.assertIsNotNone(value)


if __name__ == '__main__':
    rostest.rosrun('tutorial', 'test_params', MyTestCase)
