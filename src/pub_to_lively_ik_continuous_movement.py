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

from wisc_msgs.msg import DebugGoals
from wisc_tools.structures import Pose
import rospy
import random
import math
from scipy import interpolate
from wisc_tools.structures import Position, Quaternion, Pose, PoseTrajectory
from wisc_tools.control import StateController

class Driver(object):
    def __init__(self):
        num_poses = 5
        radius = 500 # mm
        poses = []
        times = []

        radius_squared = radius**2

        previous_pose = None

        for pose in range(num_poses):
            d = radius_squared + 1
            while(d > radius_squared):
                x = random.uniform(-radius, radius)
                y = random.uniform(-radius, radius)
                z = random.uniform(-radius, radius)
                d = x**2 + y**2 + z**2

            position = Position(x, y, z)

            orientation = Quaternion.from_euler_dict({'r': random.uniform(0, 2 * math.pi), 'p': random.uniform(0, 2 * math.pi), 'y': random.uniform(0, 2 * math.pi)})

            pose = Pose(position, orientation)

            if previous_pose is not None:
                times.append(StateController.time_to_pose(previous_pose, pose))
            else:
                times.append(0.5)

            poses.append(pose)
            previous_pose = pose

        self.pose_trajectory = PoseTrajectory([{'time': times[i], 'pose': poses[i]} for i in range(len(poses))])

        self.pose_trajectory.__interpolate__()

        self.goal_pub = rospy.Publisher('/relaxed_ik/debug_goals', DebugGoals, queue_size=10)
        self.start_time = rospy.get_time()

    def run(self):
        time = rospy.get_time() - self.start_time

        print('time: {}'.format(time))
        print(self.pose_trajectory[time])

        debug_goal = DebugGoals()

        debug_goal.ee_poses.append(self.pose_trajectory[time])

        self.goal_pub.publish(debug_goal)

if __name__ == '__main__':
    rospy.init_node('eval_lively_ik')

    driver = Driver()

    while not rospy.is_shutdown():
        driver.run()
