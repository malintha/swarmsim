#!/usr/bin/env python
PKG = 'swarmsim_testing'

import sys
import rospy
import unittest
from gazebo_msgs.msg import ModelStates

class TestSwarmSimulation(unittest.TestCase):
    # def modelStatesCB(data):
    

    def test_TwoRobotSimulation(self):
        rospy.wait_for_message("/gazebo/model_states", ModelStates, 100)
        rospy.Subscriber("/gazebo/model_states", ModelStates, callback)


        # self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_TwoRobotSimulation', TestSwarmSimulation)
