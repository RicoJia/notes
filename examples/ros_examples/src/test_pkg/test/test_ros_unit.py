#! /usr/bin/env python

import unittest
import rospy
import rostest

class MyTestCase(unittest.TestCase):
    def test_param_loaded(self):
        value = rospy.get_param('/value', None)
        self.assertIsNotNone(value)


if __name__ == '__main__':
    rostest.rosrun('tutorial', 'test_params', MyTestCase)
