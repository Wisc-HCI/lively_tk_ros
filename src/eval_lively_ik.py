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


from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from RelaxedIK.Utils.noise_utils import noise1D, clamp
from RelaxedIK.Utils import tf_fast as Tf
from RelaxedIK.Utils.filter import EMA_filter
from sensor_msgs.msg import JointState
from wisc_msgs.msg import DebugPoseAngles, DebugGoals
from wisc_tools.structures import Pose, Quaternion, Position, PoseTrajectory
from wisc_tools.control import StateController

import rospy
import itertools
import os
import tf
import numpy as np
import random
from argparse import ArgumentParser
import math

UR3E_INITIAL_POSE = Pose.from_pose_dict({'position':{'x':-0.12590331808269600,
                                                     'y':0.23734846974527900,
                                                     'z':0.3734423326681300},
                                         'quaternion':{'w':0.5046115849968640,
                                                       'x':-0.4993768344058750,
                                                       'y':0.5065220290165270,
                                                       'z':0.48931110723822800}
                                        })

def generate_goal_pose(robot,joint_limits,ee_space=False):
    if ee_space:
        radius = 0.5
        phi = random.uniform(0,2*math.pi)
        costheta = random.uniform(-1,1)
        u = random.uniform(0,1)

        theta = math.acos( costheta )
        r = radius * u**(1/3)

        x = r * math.sin( theta) * math.cos( phi )
        y = r * math.sin( theta) * math.sin( phi )
        z = abs(r * math.cos( theta ))+0.25

        position = Position(x, y, z)

        orientation = Quaternion.from_euler_dict({'r': random.uniform(0, 2 * math.pi), 'p': random.uniform(0, 2 * math.pi), 'y': random.uniform(0, 2 * math.pi)})

        pose = Pose(position, orientation)
    else:
        joint_values = [random.uniform(limit[0],limit[1]) for limit in joint_limits]
        f = robot.getFrames(joint_values)[0]
        pos = f[0][-1]
        position = Position(pos[0],pos[1],pos[2])
        new_mat = np.zeros((4, 4))
        new_mat[0:3, 0:3] = f[1][-1]
        new_mat[3, 3] = 1
        rotation = Tf.euler_from_matrix(new_mat, 'szxy')
        info = {'position':{'x':pos[0],'y':pos[1],'z':pos[2]},
                'rotation':{'r':rotation[0],'p':rotation[1],'y':rotation[2]}
               }
        pose = Pose.from_eulerpose_dict(info)
    return pose

def generate_pose_trajectory(robot,joint_limits,num_poses,ee_space=False):
    poses = [UR3E_INITIAL_POSE]
    times = [0.25]

    last_pose = poses[0]
    last_time = times[0]

    for pose_index in range(num_poses):
        pose = generate_goal_pose(robot,joint_limits,ee_space)
        time = last_time +  StateController.time_to_pose(last_pose, pose)

        times.append(time)
        last_time = time
        poses.append(pose)
        last_pose = pose

    return PoseTrajectory([{'time': times[i], 'pose': poses[i]} for i in range(len(poses))]), last_time

class Test(object):
    def __init__(self,name,robot,joint_limits):
        self.name = name
        self.robot = robot
        self.joint_limits = joint_limits
        self.running = False

    def start(self):
        rospy.loginfo("{0} Test Initializing".format(self.name))
        self.running = True

    @property
    def command(self):
        self.running = False
        dc = [0,0,0,0,0,0]
        bias = Position(1,1,1)
        weights = [25.0, 10.0, 25.0, 10.0, 5.0, 2.0, 0.1, 1.0, 2.0]
        return UR3E_INITIAL_POSE,dc,bias,weights

class ContinuousTest(Test):
    def __init__(self,robot,joint_limits,num_poses):
        super(ContinuousTest,self).__init__('Continuous',robot,joint_limits)
        self.num_poses = num_poses
        self.pose_trajectory, self.last_time = generate_pose_trajectory(robot,joint_limits,self.num_poses)
        self.start_time = 0

    def start(self):
        super(ContinuousTest,self).start()
        rospy.loginfo("Executing Continuous Trajectory with {0} waypoints, {1} seconds long".format(len(self.pose_trajectory.wps),self.last_time))
        self.start_time = rospy.get_time()

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        pose = self.pose_trajectory[time]
        if time > self.last_time:
            self.running = False
        dc = [0,0,0,0,0,0]
        bias = Position(1,1,1)
        weights = [25.0, 10.0, 25.0, 10.0, 5.0, 2.0, 0.1, 1.0, 2.0]
        return pose,dc,bias,weights

class Eval(object):
    def __init__(self,path_to_src,root="./noise"):
        self.fixed_frame = rospy.get_param('fixed_frame')
        self.ee_fixed_joints = rospy.get_param('ee_fixed_joints')
        self.starting_config = rospy.get_param('starting_config')
        self.joint_names = rospy.get_param('joint_names')
        self.joint_ordering = rospy.get_param('joint_ordering')
        self.joint_limits = rospy.get_param('joint_limits')
        self.n_joints = len(self.joint_ordering)
        self.n_arms = len(self.ee_fixed_joints)
        self.joint_scaling = [.001*abs(val[1]-val[0]) for val in self.joint_limits]
        self.joint_seeds = [random.randint(0,10240) for joint in self.joint_ordering]
        self.last_random = [0]*self.n_joints
        self.last_random_ewma = [0]*self.n_joints
        self.ewma_filter = EMA_filter([0]*self.n_joints,.1,75)
        self.algorithms = ['relaxed','lively','joint_perlin','joint_random_normal_ewma']
        self.last_time = 0
        self.running = False
        self.seq = 0

        # Clear the current contents if they exist
        with open(root+"_joints.csv","w") as js_file:
            header = ['time','algorithm','collision','eval']+self.joint_ordering
            # for group in ['relaxed','lively','joint_perlin','joint_random_normal','joint_random_normal_ewma']:
            #     header.extend([j+"_"+group for j in self.joint_ordering])
            js_file.write(",".join(header)+"\n")
        with open(root+"_poses.csv","w") as pose_file:
            values = ['x','y','z','qx','qy','qz','qw']
            header = ['time','algorithm','collision','eval']
            # for group in ['goal','relaxed','lively','joint_perlin','joint_random_normal','joint_random_normal_ewma']:
            #     header.extend(["{0}_{1}_{2}".format(pair[0],pair[1],group) for pair in itertools.product(self.ee_fixed_joints,values)])
            pose_file.write(",".join(header+values)+"\n")
        self.js_file_writer = open(root+"_joints.csv",mode='a')
        self.pose_file_writer = open(root+"_poses.csv",mode='a')

        self.vars = RelaxedIK_vars('relaxedIK',
                                   path_to_src + '/RelaxedIK/urdfs/' + rospy.get_param('urdf_file_name'),
                                   self.joint_names,
                                   rospy.get_param('ee_fixed_joints'),
                                   self.joint_ordering,
                                   init_state=rospy.get_param('starting_config'),
                                   collision_file=rospy.get_param('collision_file_name'),
                                   pre_config=True,
                                   position_mode='absolute',
                                   rotation_mode='absolute',
                                   path_to_src=path_to_src)
        self.robot = self.vars.robot
        self.collision_graph = self.vars.collision_graph

        self.tests = [
            ContinuousTest(self.robot,self.joint_limits,10),
        ]

        self.goal_pub = rospy.Publisher('/relaxed_ik/debug_goals', DebugGoals, queue_size=10)
        self.dpa_sub = rospy.Subscriber('/relaxed_ik/debug_pose_angles',
                                        DebugPoseAngles,
                                        self.dpa_sub_cb,
                                        queue_size=5)

    def start(self):
        rospy.loginfo("Initializing Driver")
        self.start_time = rospy.get_time()
        if len(self.tests) > 0:
            self.tests[0].start()
        self.running = True

    def dpa_sub_cb(self,dpa_msg):
        # rospy.loginfo("Got Update!")
        if dpa_msg.eval_type == 'exit':
            self.running = False
        elif dpa_msg.eval_type != 'null':
            time = dpa_msg.header.stamp.to_sec()

            # Read the DebugPoseGoal messages,
            # and Use forward finematics to deduce
            # the poses that would result
            joints = self.n_joints*5*[0]
            perlin_noise = [(noise1D(time/4,self.joint_seeds[i]))*self.joint_scaling[i] for i in range(self.n_joints)]
            #print(perlin_noise)
            normal_delta = [random.normalvariate(-1*self.last_random[i]*.01, self.joint_scaling[i]*.03) for i in range(self.n_joints)]
            self.last_random = [normal_delta[i]+self.last_random[i] for i in range(self.n_joints)]
            self.last_random_ewma = self.ewma_filter.filter(self.last_random)

            joints = {algorithm:[] for algorithm in self.algorithms}
            collision = {algorithm:0 for algorithm in self.algorithms}
            frames = {algorithm:None for algorithm in self.algorithms}
            # First handle the vanilla relaxed_ik values
            joints['relaxed'] =dpa_msg.angles_relaxed
            joints['lively'] = dpa_msg.angles_lively
            joints['joint_perlin'] = [clamp(angle+perlin_noise[i],self.joint_limits[i][0],self.joint_limits[i][1]) for i,angle in enumerate(dpa_msg.angles_relaxed)]
            joints['joint_random_normal'] = [clamp(angle+self.last_random[i],self.joint_limits[i][0],self.joint_limits[i][1]) for i,angle in enumerate(dpa_msg.angles_relaxed)]
            joints['joint_random_normal_ewma'] = [clamp(angle+self.last_random_ewma[i],self.joint_limits[i][0],self.joint_limits[i][1]) for i,angle in enumerate(dpa_msg.angles_relaxed)]

            for algorithm in self.algorithms:
                frames[algorithm] = self.robot.getFrames(joints[algorithm])
                collision[algorithm] = self.collision_graph.get_collision_score(frames[algorithm])
                self.js_file_writer.write(",".join([str(time),algorithm,str(collision[algorithm]),dpa_msg.eval_type]+\
                                                   [str(j) for j in joints[algorithm]])+"\n")

            # Transfer the poses from the actual goal
            for i in range(self.n_arms):
                pos = dpa_msg.ee_poses[i].position
                ori = dpa_msg.ee_poses[i].orientation
                info = [time,'goal',None,dpa_msg.eval_type,pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w]
            self.pose_file_writer.write(",".join([str(d) for d in info])+"\n")

            # Transfer the "ideal" noise
            for i in range(self.n_arms):
                pos = dpa_msg.ideal_noise[i].position
                ori = dpa_msg.ideal_noise[i].orientation
                info = [time,'ideal',None,dpa_msg.eval_type,pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w]
            self.pose_file_writer.write(",".join([str(d) for d in info])+"\n")

            # Solve the positions that result for relaxed_ik and lively_k
            for algorithm in self.algorithms:
                for f in frames[algorithm]:
                    pos = f[0][-1]
                    new_mat = np.zeros((4, 4))
                    new_mat[0:3, 0:3] = f[1][-1]
                    new_mat[3, 3] = 1
                    rotation = Tf.euler_from_matrix(new_mat, 'szxy')
                    info = {'r':rotation[0],'p':rotation[1],'y':rotation[2]}
                    ori = Quaternion.from_euler_dict(info).ros_quaternion
                    info = [time,algorithm,collision[algorithm],dpa_msg.eval_type,pos[0],pos[1],pos[2],ori.x,ori.y,ori.z,ori.w]
                    self.pose_file_writer.write(",".join([str(d) for d in info])+"\n")

    def run(self):
        time = rospy.get_time() - self.start_time

        debug_goal = DebugGoals()
        debug_goal.header.seq =self.seq
        debug_goal.header.stamp = rospy.get_rostime()

        if self.tests[0].running:
            pose,dc,bias,weights = self.tests[0].command
            test_name = self.tests[0].name
        else:
            rospy.loginfo("Test Finished. Clearing Test...")
            self.tests.pop(0)
            if len(self.tests) == 0:
                rospy.loginfo("No more tests to run. Exiting...")
                self.running = False
                pose = UR3E_INITIAL_POSE
                dc = [0,0,0,0,0,0]
                bias = Position(1,1,1)
                weights = [25.0, 10.0, 25.0, 10.0, 5.0, 2.0, 0.1, 1.0, 2.0]
                test_name = 'exit'
            else:
                rospy.loginfo("More tests to run. Starting...")
                self.tests[0].start()
                pose,dc,bias,weights = self.tests[0].command
                test_name = self.tests[0].name




        debug_goal.ee_poses.append(pose.ros_pose)
        debug_goal.eval_type = test_name
        debug_goal.dc_values = dc
        debug_goal.bias = bias.ros_point

        self.goal_pub.publish(debug_goal)
        self.seq += 1

if __name__ == '__main__':
    rospy.init_node('eval_lively_ik')

    parser = ArgumentParser(description='Eval of Lively IK')
    parser.add_argument("--file_root", dest="file_root", default="./noise", help="Specify the root name for files")
    args = parser.parse_known_args()[0]

    path_to_src = os.path.dirname(__file__)
    initialized = False
    try:
        urdf_file_name = rospy.get_param('urdf_file_name')
        urdf_file = open(path_to_src + '/RelaxedIK/urdfs/' + urdf_file_name, 'r')
        urdf_string = urdf_file.read()
        rospy.set_param('robot_description', urdf_string)


    except Exception as e:
        rospy.logerr('Could not retrieve/apply required parameters!')
        rospy.logerr(str(e))
    evaluator = Eval(path_to_src,args.file_root)
    initialized = True

    while not rospy.get_param("ready"):
        rospy.sleep(1)

    rate = rospy.Rate(40)
    evaluator.start()

    while not rospy.is_shutdown() and evaluator.running:
        evaluator.run()
        rate.sleep()

    try:
        evaluator.js_file_writer.close()
        evaluator.pose_file_writer.close()
    except:
        if initialized:
            rospy.logerr('Could not close files!')

    rospy.loginfo("Finished Publishing")
