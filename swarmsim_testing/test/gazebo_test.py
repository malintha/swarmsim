#!/usr/bin/env python
PKG = 'swarmsim_testing'

import sys
import unittest
from sensor_msgs import NavSatFix

## A sample python unit test
class TestBareBones(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
        image_msg = rospy.wait_for_message("/0/mavros/global_position/global", NavSatFix, 100)
        self.assertEquals(1, 1, "1!=1")
        print("## running test ##")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones)
