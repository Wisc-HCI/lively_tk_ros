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
        num_poses = 10
        radius = .5 # m
        poses = []
        times = []

        radius_squared = radius**2

        previous_pose = None
        self.last_time = 0

        for pose in range(num_poses):
            # d = radius_squared + 1
            # while(d > radius_squared):
            #     x = random.uniform(-radius, radius)
            #     y = random.uniform(-radius, radius)
            #     z = random.uniform(0 , radius)
            #     d = x**2 + y**2 + z**2
            phi = random.uniform(0,2*math.pi)
            costheta = random.uniform(-1,1)
            u = random.uniform(0,1)

            theta = math.acos( costheta )
            r = radius * u**(1/3)

            x = r * math.sin( theta) * math.cos( phi )
            y = r * math.sin( theta) * math.sin( phi )
            z = abs(r * math.cos( theta ))

            position = Position(x, y, z)

            orientation = Quaternion.from_euler_dict({'r': random.uniform(0, 2 * math.pi), 'p': random.uniform(0, 2 * math.pi), 'y': random.uniform(0, 2 * math.pi)})

            pose = Pose(position, orientation)

            if previous_pose is not None:
                time = 4*StateController.time_to_pose(previous_pose, pose)
            else:
                time = 0.5

            times.append(time)
            self.last_time += time

            poses.append(pose)
            previous_pose = pose

        self.pose_trajectory = PoseTrajectory([{'time': times[i], 'pose': poses[i]} for i in range(len(poses))])

        #self.pose_trajectory.__interpolate__()

        self.goal_pub = rospy.Publisher('/relaxed_ik/debug_goals', DebugGoals, queue_size=10)
        self.seq = 0
        self.start_time = None
        self.running = False

    def start(self):
        rospy.loginfo("Initializing Driver")
        self.start_time = rospy.get_time()
        self.running = True

    def run(self):
        time = rospy.get_time() - self.start_time

        # print('time: {}'.format(time))
        # print(self.pose_trajectory[time])

        debug_goal = DebugGoals()
        debug_goal.header.seq =self.seq
        debug_goal.header.stamp = rospy.get_rostime()

        debug_goal.ee_poses.append(self.pose_trajectory[time].ros_pose)
        debug_goal.eval_type = "continuous"
        debug_goal.dc_values = [0,0,0,0,0,0]
        debug_goal.bias = Position(1,1,1).ros_point

        self.goal_pub.publish(debug_goal)
        self.seq += 1
        if time > self.last_time:
            self.running = False

if __name__ == '__main__':
    rospy.init_node('lively_ik_eval_generator')

    driver = Driver()

    while not rospy.get_param("ready"):
        rospy.sleep(1)

    rate = rospy.Rate(40)
    driver.start()
    while not rospy.is_shutdown() and driver.running:
        driver.run()
        rate.sleep()

    rospy.loginfo("Finished Publishing")
