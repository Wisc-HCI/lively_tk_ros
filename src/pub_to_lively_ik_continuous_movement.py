#! /usr/bin/env python
'''
author: Andrew Schoen
website: http://pages.cs.wisc.edu/~schoen/
email: schoen@cs.wisc.edu
last update: 02/13/2020

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''
######################################################################################################

from wisc_msgs.msg import EEPoseGoals
from wisc_tools.structures import Pose
import rospy

class Driver(object):
    def __init__(self):
        self.joint_names = rospy.get_param('joint_names')
        # TODO: Define the bounding box (500 mm radius)
        # TODO: Use an algorithm to define a smoothly interpolated circuit within the bounding sphere.

        # Define a pub for ee pose goals
        self.goal_pub = rospy.Subscriber('/relaxed_ik/ee_pose_goals',EEPoseGoals, queue_size=5)

    def run():
        # TODO: interpolate and publish what you need:
        pass


if __name__ == '__main__':
    rospy.init_node('eval_lively_ik')

    driver = Driver()

    while not rospy.is_shutdown():
        driver.run()
