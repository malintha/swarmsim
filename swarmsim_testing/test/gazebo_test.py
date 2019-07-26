#!/usr/bin/env python
PKG = 'swarmsim_testing'

import sys
import rospy
import unittest
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import WaypointList

class TestSwarmSimulation(unittest.TestCase):
    def test_TwoRobotSimulation(self):

        rospy.wait_for_message("/1/mavros/mission/waypoints", WaypointList, 100)
        print("######")
        self.assertEquals(1, 1, "1!=1")
        print("## running test ##")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_TwoRobotSimulation', TestSwarmSimulation)
