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
from wisc_tools.structures import Pose, Quaternion, Position, PoseTrajectory, ModeTrajectory
from wisc_tools.control import StateController
from wisc_tools.convenience import ProgressBar
from opensimplex import OpenSimplex
# from bullet.bullet import BulletSimulator

import rospy
import itertools
import os
import tf
import numpy as np
import random
from argparse import ArgumentParser
import math


BIAS = {'ur3e':{'biased':Position(0.5,1,0.5),'unbiased':Position(1,1,1)},
        'nao_v4':{'biased':Position(0.5,1,0.5),'unbiased':Position(1,1,1)}
       }

WEIGHT_DEFAULTS = {'ur3e':{'normal':[10.0, 8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 3.0, 0.2, 1.0, 7.0],
                           'lively':[ 2.0, 1.6, 8.0, 6.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 3.0, 0.2, 1.0, 7.0]
                          },
                   'nao_v4':{'normal':[1.0, 6.0, 6.0, 6.0, 6.0, 1.0, 4.0, 4.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 2.0, 0.1, 1.0, 2.0, 10.0, 10.0, 10.0],
                             'lively':[1.0, 6.0, 6.0, 6.0, 6.0, 1.0, 4.0, 4.0, 5.0, 5.0, 5.0, 5.0, 0.5, 0.5, 0.5, 5.0, 5.0, 5.0, 2.0, 0.1, 1.0, 2.0, 10.0, 10.0, 10.0]
                            }
                  }

DC_DEFAULTS = {'ur3e':{'normal':[1.55, -1.77, 1.4, -1.19, -1.57, 0.0],
                       'lively':[1.55, -1.77, 1.4, -1.19, -1.57, 0.0]
                      },
               'nao_v4':{'normal':[0.0,0.0, 1.5, 0.15,0.0,-0.04,-1.3, 1.5,-0.15,0.0,0.04, 1.3, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,.4,.4],
                         'lively':[0.0,0.0, 1.5, 0.15,0.0,-0.04,-1.3, 1.5,-0.15,0.0,0.04, 1.3, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,.4,.4]
                        }
              }


JOINT_SCALES = {'ur3e':{'elbow_joint': 0.04943172615553592,
                        'shoulder_lift_joint': 0.06519351163038933,
                        'shoulder_pan_joint': 0.05981500545991754,
                        'wrist_1_joint': 0.060635328273466516,
                        'wrist_2_joint': 0.03854340782800793,
                        'wrist_3_joint': 0.08667862514476901},
                'nao_v4':{'HeadPitch': 0.03241025901778748,
                         'HeadYaw': 0.03487730221459542,
                         'LAnklePitch': 0.01959773277218994,
                         'LAnkleRoll': 0.017679353837950794,
                         'LElbowRoll': 0.006171122449291864,
                         'LElbowYaw': 0.07014936768535909,
                         'LHand': 0.06780014567033442,
                         'LHipPitch': 0.019179325881739477,
                         'LHipRoll': 0.03467096850855242,
                         'LHipYawPitch': 0.035268050925749216,
                         'LKneePitch': 0.0039363565040527355,
                         'LShoulderPitch': 0.01499393991605014,
                         'LShoulderRoll': 0.012639605328994576,
                         'LWristYaw': 0.06573754145897988,
                         'RAnklePitch': 0.02066244887363038,
                         'RAnkleRoll': 0.01746599550689567,
                         'RElbowRoll': 0.01501447245298513,
                         'RElbowYaw': 0.09027797095715812,
                         'RHand': 0.0704930839703507,
                         'RHipPitch': 0.021021206596846108,
                         'RHipRoll': 0.03479924641126292,
                         'RHipYawPitch': 0.035266210267831555,
                         'RKneePitch': 0.014729152274376534,
                         'RShoulderPitch': 0.019397132143749468,
                         'RShoulderRoll': 0.017491677632176367,
                         'RWristYaw': 0.07781985791058026}
        }

INITIAL_JOINTS = {'ur3e':[1.55, -1.77, 1.4, -1.19, -1.57, 0.0],
                  'nao_v4':[0,0, 1.5, 0.15,0,-0.04,-1.3, 1.5,-0.15,0,0.04, 1.3, 0,0,0,0,0,0, 0,0,0,0,0,0,.4,.4]
                 }

ARM_ACTIONS = {'ur3e':[],
               'nao_v4':[
                    {'time':0,    'joints':[0,0, 1.5, 0.15,0,-0.04,-1.3,  1.5,-0.15,   0,0.04, 1.3, 0,0,0,0,0,0, 0,0,0,0,0,0,.4,.4]},
                    {'time':4,    'joints':[0,0, 1.5, 0.15,0,-0.04,-1.3,  1.5,-0.15,   0,0.04, 1.3, 0,0,0,0,0,0, 0,0,0,0,0,0,.4,.4]},
                    {'time':8,    'joints':[0,0, 1.5, 0.15,0,-0.04,-1.3, 0.59,-0.68,1.18,0.77, 0.1, 0,0,0,0,0,0, 0,0,0,0,0,0,.4,.4]},
                    {'time':12.5, 'joints':[0,0, 1.5, 0.15,0,-0.04,-1.3,-0.49,-0.68,1.18,1.25, 0.1, 0,0,0,0,0,0, 0,0,0,0,0,0,.4,.4]},
                    {'time':13,   'joints':[0,0, 1.5, 0.15,0,-0.04,-1.3,-0.49,-0.68,1.18,0.77, 0.1, 0,0,0,0,0,0, 0,0,0,0,0,0,.4,.4]},
                    {'time':13.5, 'joints':[0,0, 1.5, 0.15,0,-0.04,-1.3,-0.49,-0.68,1.18,1.25, 0.1, 0,0,0,0,0,0, 0,0,0,0,0,0,.4,.4]},
                    {'time':17,   'joints':[0,0, 1.5, 0.15,0,-0.04,-1.3, 0.59,-0.68,1.18,0.77, 0.1, 0,0,0,0,0,0, 0,0,0,0,0,0,.4,.4]},
                    {'time':21,   'joints':[0,0, 1.5, 0.15,0,-0.04,-1.3,  1.5,-0.15,   0,0.04, 1.3, 0,0,0,0,0,0, 0,0,0,0,0,0,.4,.4]},
                    {'time':24,   'joints':[0,0, 1.5, 0.15,0,-0.04,-1.3,  1.5,-0.15,   0,0.04, 1.3, 0,0,0,0,0,0, 0,0,0,0,0,0,.4,.4]}
               ]
              }

def js_to_poses(robot,joints):
    frames = robot.getFrames(joints)
    poses = []
    for f in frames:
        pos = f[0][-1]
        new_mat = np.zeros((4, 4))
        new_mat[0:3, 0:3] = f[1][-1]
        new_mat[3, 3] = 1
        rotation = Tf.euler_from_matrix(new_mat, 'szxy')
        info = {'position':{'x':pos[0],'y':pos[1],'z':pos[2]},
                'rotation':{'r':rotation[0],'p':rotation[1],'y':rotation[2]}
               }
        poses.append(Pose.from_eulerpose_dict(info))
    return poses

def generate_goal_poses(robot,joint_limits,collision_graph):
    passed = False
    while not passed:
        joint_values = [random.uniform(limit[0],limit[1]) for limit in joint_limits]
        frames = robot.getFrames(joint_values)
        passed = collision_graph.get_collision_score(frames) < 4.0
    poses = js_to_poses(robot,joint_values)
    return poses,joint_values

def generate_pose_trajectory(robot,joint_limits,collision_graph,num_poses,ee_space=False):
    poses = [UR3E_INITIAL_POSE]
    times = [0.25]

    last_pose = poses[0]
    last_time = times[0]

    for pose_index in range(num_poses):
        pose,joints = generate_goal_poses(robot,joint_limits,collision_graph)
        time = last_time +  StateController.time_to_pose(last_pose, pose)

        times.append(time)
        last_time = time
        poses.append(pose)
        last_pose = pose

    return PoseTrajectory([{'time': times[i], 'pose': poses[i]} for i in range(len(poses))],kind='cubic'), last_time

class Test(object):
    def __init__(self,name,i,robot,robot_name,joint_limits,collision_graph):
        self.name = name
        self.i = i
        self.robot = robot
        self.robot_name = robot_name
        self.joint_limits = joint_limits
        self.collision_graph = collision_graph
        self.running = False
        self.started = False

    def start(self):
        rospy.loginfo("{0} Test Initializing".format(self.name))
        self.running = True
        self.started = True

    @property
    def initial(self):
        return self.command

    @property
    def command(self):
        self.running = False
        poses = js_to_poses(self.robot,INITIAL_JOINTS[self.robot_name])
        dc = DC_DEFAULTS[self.robot_name]['lively']
        bias = BIAS[self.robot_name]['unbiased']
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        return poses,dc,bias,normal_weights,lively_weights

class ContinuousTest(Test):
    def __init__(self,i,robot,robot_name,joint_limits,collision_graph,num_poses):
        super(ContinuousTest,self).__init__('Continuous',i,robot,robot_name,joint_limits,collision_graph)
        self.num_poses = num_poses
        self.pose_trajectories = []
        self.last_time = 0
        for arm in range(self.robot.numChains):
            pose_trajectory, last_time = generate_pose_trajectory(self.robot,self.joint_limits,self.collision_graph,self.num_poses)
            self.pose_trajectories.append(pose_trajectory)
            if last_time > self.last_time:
                self.last_time = last_time
        self.start_time = 0

    def start(self):
        super(ContinuousTest,self).start()
        rospy.loginfo("Executing Continuous Trajectory Test {0} with {1} waypoints for {2} seconds.".format(self.i,len(self.pose_trajectory.wps),self.last_time))
        self.start_time = rospy.get_time()

    @property
    def initial(self):
        poses = [pose_trajectory[0] for pose_trajectory in self.pose_trajectories]
        dc = DC_DEFAULTS[self.robot_name]['lively']
        bias = BIAS[self.robot_name]['unbiased']
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        return poses,dc,bias,normal_weights,lively_weights

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        if time > self.last_time:
            self.running = False
        ProgressBar(time, self.last_time, prefix = self.name, suffix = '', decimals = 1, length = 100, fill = '=')
        poses = [pose_trajectory[time] for pose_trajectory in self.pose_trajectories]
        dc = DC_DEFAULTS[self.robot_name]['lively']
        bias = BIAS[self.robot_name]['unbiased']
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        return poses,dc,bias,normal_weights,lively_weights

class StaticInitialTest(Test):
    def __init__(self,i,robot,robot_name,joint_limits,collision_graph,duration,biased=False):
        super(StaticInitialTest,self).__init__('StaticInitial',i,robot,robot_name,joint_limits,collision_graph)
        self.poses = js_to_poses(self.robot,INITIAL_JOINTS[self.robot_name])
        # print("StaticInitial")
        # for pose in self.poses:
        #     euler = pose.quaternion.dict
        #     print(((pose.position.x,pose.position.y,pose.position.z),(euler['r'],euler['p'],euler['y'])))
        #     print(pose.quaternion)
        self.last_time = duration
        self.start_time = 0
        self.in_buffer = True
        self.bias_setting = 'biased' if biased else 'unbiased'

    def start(self):
        super(StaticInitialTest,self).start()
        rospy.loginfo("Executing Static Initial Test {0} for {1} seconds.".format(self.i,self.last_time))
        self.start_time = rospy.get_time()

    @property
    def initial(self):
        poses = self.poses
        dc = DC_DEFAULTS[self.robot_name]['lively']
        bias = BIAS[self.robot_name][self.bias_setting]
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        return poses,dc,bias,normal_weights,lively_weights

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        ProgressBar(time, self.last_time, prefix = self.name, suffix = '', decimals = 1, length = 100, fill = '=')
        if time > self.last_time:
            self.running = False
        dc = DC_DEFAULTS[self.robot_name]['lively']
        bias = BIAS[self.robot_name][self.bias_setting]
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        return self.poses,dc,bias,normal_weights,lively_weights

class ConstrainedMovementTest(Test):
    def __init__(self,i,robot,robot_name,joint_limits,collision_graph,duration):
        super(ConstrainedMovementTest,self).__init__('ConstrainedMovement',i,robot,robot_name,joint_limits,collision_graph)
        self.poses,self.joints = generate_goal_poses(self.robot,self.joint_limits,self.collision_graph)
        self.last_time = duration
        self.start_time = 0
        self.in_buffer = True

    def start(self):
        super(ConstrainedMovementTest,self).start()
        rospy.loginfo("Executing Constrianed Movement Test {0} for {1} seconds.".format(self.i,self.last_time))
        self.start_time = rospy.get_time()

    @property
    def initial(self):
        poses = self.poses
        dc = self.joints
        bias = BIAS[self.robot_name]['unbiased']
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights[6] = 2
        return poses,dc,bias,normal_weights,lively_weights

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        ProgressBar(time, self.last_time, prefix = self.name, suffix = '', decimals = 1, length = 100, fill = '=')
        if time > self.last_time:
            self.running = False
        dc = INITIAL_JOINTS[self.robot_name]
        bias = BIAS[self.robot_name]['unbiased']
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights[4] = 3
        return self.poses,dc,bias,normal_weights,lively_weights

class WaveTest(Test):
    def __init__(self,i,robot,robot_name,joint_limits,collision_graph,biased=False):
        super(WaveTest,self).__init__('Action',i,robot,robot_name,joint_limits,collision_graph)

class ActionTest(Test):
    def __init__(self,i,robot,robot_name,joint_limits,collision_graph,biased=False):
        super(ActionTest,self).__init__('Action',i,robot,robot_name,joint_limits,collision_graph)
        poses = [action_pose['joints'] for action_pose in ARM_ACTIONS[self.robot_name]]
        #print([len(poses[i]) for i in range(len(poses))])
        self.action_frames = [js_to_poses(self.robot,poses[i]) for i in range(len(poses))]
        self.times = [action_pose['time'] for action_pose in ARM_ACTIONS[self.robot_name]]
        self.pose_trajectories = []
        self.num_arms = len(self.action_frames[0])
        for i in range(self.num_arms):
            self.pose_trajectories.append(PoseTrajectory([{'time':self.times[j],'pose':self.action_frames[j][i]} for j in range(len(self.times))],kind='cubic'))
        self.last_time = self.times[-1]
        self.start_time = 0
        self.in_buffer = True
        self.bias_setting = 'biased' if biased else 'unbiased'

    def start(self):
        super(ActionTest,self).start()
        rospy.loginfo("Executing Action Test {0} for {1} seconds.".format(self.i,self.last_time))
        self.start_time = rospy.get_time()

    @property
    def initial(self):
        poses = [self.pose_trajectories[i][0] for i in range(self.num_arms)]
        dc = DC_DEFAULTS[self.robot_name]['lively']
        bias = BIAS[self.robot_name][self.bias_setting]
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        return poses,dc,bias,normal_weights,lively_weights

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        poses = [self.pose_trajectories[i][time] for i in range(self.num_arms)]
        ProgressBar(time, self.last_time, prefix = self.name, suffix = '', decimals = 1, length = 100, fill = '=')
        if time > self.last_time:
            self.running = False
        dc = DC_DEFAULTS[self.robot_name]['lively']
        bias = BIAS[self.robot_name][self.bias_setting]
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        return poses,dc,bias,normal_weights,lively_weights

class StaticDescriptiveTest(Test):
    def __init__(self,i,robot,robot_name,joint_limits,collision_graph,duration,biased=False):
        super(StaticDescriptiveTest,self).__init__('StaticDescriptive',i,robot,robot_name,joint_limits,collision_graph)
        self.poses,self.joints = generate_goal_poses(self.robot,self.joint_limits,self.collision_graph)
        self.last_time = duration
        self.start_time = 0
        self.in_buffer = True
        self.bias_setting = 'biased' if biased else 'unbiased'

    def start(self):
        super(StaticDescriptiveTest,self).start()
        rospy.loginfo("Executing Static Descriptive Test {0} for {1} seconds.".format(self.i,self.last_time))
        self.start_time = rospy.get_time()

    @property
    def initial(self):
        poses = self.poses
        dc = self.joints
        bias = BIAS[self.robot_name][self.bias_setting]
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        return poses,dc,bias,normal_weights,lively_weights

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        ProgressBar(time, self.last_time, prefix = self.name, suffix = '', decimals = 1, length = 100, fill = '=')
        if time > self.last_time:
            self.running = False
        dc = self.joints
        bias = BIAS[self.robot_name][self.bias_setting]
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        return self.poses,dc,bias,normal_weights,lively_weights

class DynamicPriorityTest(Test):
    def __init__(self,i,robot,robot_name,joint_limits,collision_graph,duration):
        super(DynamicPriorityTest,self).__init__('DynamicPriority',i,robot,robot_name,joint_limits,collision_graph)
        self.poses = js_to_poses(self.robot,INITIAL_JOINTS[self.robot_name])
        self.last_time = duration
        self.lively_weight = ModeTrajectory([{'time':0,'mode':8},{'time':duration/4,'mode':8},{'time':3*duration/4,'mode':0},{'time':duration,'mode':0}])
        self.start_time = 0
        self.in_buffer = True

    def start(self):
        super(DynamicPriorityTest,self).start()
        rospy.loginfo("Executing Dynamic Priority Test {0} for {1} seconds.".format(self.i,self.last_time))
        self.start_time = rospy.get_time()

    @property
    def initial(self):
        poses = self.poses
        dc = INITIAL_JOINTS[self.robot_name]['lively']
        bias = BIAS[self.robot_name]['unbiased']
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        lively_weights[2] = self.lively_weight[0]
        lively_weights[3] = self.lively_weight[0]
        return poses,dc,bias,normal_weights,lively_weights

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        ProgressBar(time, self.last_time, prefix = self.name, suffix = '', decimals = 1, length = 100, fill = '=')
        if time > self.last_time:
            self.running = False
        dc = DC_DEFAULTS[self.robot_name]['lively']
        bias = BIAS[self.robot_name]['unbiased']
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        lively_weights[2] = self.lively_weight[time]
        lively_weights[3] = self.lively_weight[time]
        return self.poses,dc,bias,normal_weights,lively_weights

class DynamicFrequencyTest(Test):
    def __init__(self,i,robot,robot_name,joint_limits,collision_graph,duration):
        super(DynamicFrequencyTest,self).__init__('DynamicFrequency',i,robot,robot_name,joint_limits,collision_graph)
        self.poses,self.joints = generate_goal_poses(self.robot,self.joint_limits,self.collision_graph)
        self.last_time = duration
        self.fast_weight = ModeTrajectory([{'time':0,'mode':0},{'time':duration/3,'mode':0},{'time':2*duration/3,'mode':8},{'time':duration,'mode':8}])
        self.slow_weight = ModeTrajectory([{'time':0,'mode':8},{'time':duration/3,'mode':8},{'time':2*duration/3,'mode':0},{'time':duration,'mode':0}])
        self.start_time = 0
        self.in_buffer = True

    def start(self):
        super(DynamicFrequencyTest,self).start()
        rospy.loginfo("Executing Dynamic Frequency Test {0} for {1} seconds.".format(self.i,self.last_time))
        self.start_time = rospy.get_time()

    @property
    def initial(self):
        poses = self.poses
        dc = self.joints
        bias = BIAS[self.robot_name]['unbiased']
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        lively_weights[2] = self.slow_weight[0]
        lively_weights[3] = self.fast_weight[0]
        lively_weights[4] = 0.0
        return poses,dc,bias,normal_weights,lively_weights

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        ProgressBar(time, self.last_time, prefix = self.name, suffix = '', decimals = 1, length = 100, fill = '=')
        if time > self.last_time:
            self.running = False
        dc = self.joints
        bias = BIAS[self.robot_name]['unbiased']
        normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
        lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
        lively_weights[2] = self.slow_weight[time]
        lively_weights[3] = self.fast_weight[time]
        lively_weights[4] = 0.0
        return self.poses,dc,bias,normal_weights,lively_weights

class Eval(object):
    def __init__(self,path_to_src,root="./noise", buffer_time=10):
        self.fixed_frame = rospy.get_param('fixed_frame')
        self.ee_fixed_joints = rospy.get_param('ee_fixed_joints')
        self.starting_config = rospy.get_param('starting_config')
        self.joint_names = rospy.get_param('joint_names')
        self.joint_ordering = rospy.get_param('joint_ordering')
        self.joint_limits = rospy.get_param('joint_limits')
        self.robot_name = rospy.get_param('urdf_file_name').replace('.urdf','')
        self.n_joints = len(self.joint_ordering)
        self.n_arms = len(self.ee_fixed_joints)
        self.joint_scaling = [JOINT_SCALES[self.robot_name][joint] for joint in self.joint_ordering]
        self.joint_seeds = [random.randint(0,10240) for joint in self.joint_ordering]
        self.joint_noise_generators = [OpenSimplex(seed=seed) for seed in self.joint_seeds]
        self.last_random = [0]*self.n_joints
        self.last_random_ewma = [0]*self.n_joints
        self.ewma_filter = EMA_filter([0]*self.n_joints,.01,250)
        self.algorithms = ['relaxed','lively','joint_perlin']
        self.last_time = 0
        self.running = False
        self.finished = False
        self.buffering = False
        self.buffer_time = buffer_time
        self.buffer_start_time = 0
        self.seq = 0
        # self.bullet_sim = BulletSimulator('ur3e')

        # Clear the current contents if they exist
        with open(root+"_joints.csv","w") as js_file:
            header = ['time','algorithm','collision','eval','i']+self.joint_ordering
            # for group in ['relaxed','lively','joint_perlin','joint_random_normal','joint_random_normal_ewma']:
            #     header.extend([j+"_"+group for j in self.joint_ordering])
            js_file.write(",".join(header)+"\n")
        with open(root+"_poses.csv","w") as pose_file:
            values = ['x','y','z','qx','qy','qz','qw']
            header = ['time','algorithm','collision','eval','i','arm']
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

        self.tests = []



        for i in range(1):
            self.tests.append(StaticDescriptiveTest(i,self.robot,self.robot_name,self.joint_limits,self.collision_graph,120))

        for i in range(0):
            self.tests.append(StaticInitialTest(i,self.robot,self.robot_name,self.joint_limits,self.collision_graph,120))

        for i in range(0):
            self.tests.append(ActionTest(i,self.robot,self.robot_name,self.joint_limits,self.collision_graph))

        # self.tests.append(DynamicFrequencyTest(0,self.robot,self.robot_name,self.joint_limits,self.collision_graph,60))
        # self.tests.append(StaticInitialTest(0,self.robot,self.robot_name,self.joint_limits,self.collision_graph,60,biased=False))
        # self.tests.append(StaticInitialTest(1,self.robot,self.robot_name,self.joint_limits,self.collision_graph,60,biased=True))
        # self.tests.append(DynamicPriorityTest(0,self.robot,self.robot_name,self.joint_limits,self.collision_graph,60))
        for i in range(0):
            self.tests.append(ConstrainedMovementTest(i,self.robot,self.robot_name,self.joint_limits,self.collision_graph,60))


        self.goal_pub = rospy.Publisher('/relaxed_ik/debug_goals', DebugGoals, queue_size=10)
        self.dpa_sub = rospy.Subscriber('/relaxed_ik/debug_pose_angles',
                                        DebugPoseAngles,
                                        self.dpa_sub_cb,
                                        queue_size=5)

    def start(self):
        rospy.loginfo("Initializing Driver")
        self.start_time = rospy.get_time()
        self.running = True
        self.buffering = True
        self.buffer_start_time = self.start_time
        rospy.loginfo("Starting Buffer...")

    def dpa_sub_cb(self,dpa_msg):
        #rospy.loginfo("Got Update!")
        if dpa_msg.eval_type == 'exit' or not self.running:
            self.running = False
        elif dpa_msg.eval_type == 'buffer' or dpa_msg.eval_type == 'null':
            pass
            #rospy.loginfo('buffer or null')
        elif dpa_msg.eval_type != 'null':
            time = dpa_msg.header.stamp.to_sec()
            # rospy.loginfo('writing eval')
            # Read the DebugPoseGoal messages,
            # and Use forward finematics to deduce
            # the poses that would result
            # rospy.loginfo(dpa_msg.eval_type)
            #perlin_noise = [self.joint_noise_generators[i].noise3d(time/4,self.joint_seeds[i],500*math.sin(time/2000))*(self.joint_scaling[i]/0.3) for i in range(self.n_joints)]
            #normal_delta = [random.normalvariate(-1*self.last_random[i]*.001, self.joint_scaling[i]) for i in range(self.n_joints)]
            #self.last_random = [normal_delta[i]+self.last_random[i] for i in range(self.n_joints)]
            #self.last_random_ewma = self.ewma_filter.filter(self.last_random)
            #rospy.loginfo('defining dicts')
            joints = {algorithm:[] for algorithm in self.algorithms}
            collision = {algorithm:0 for algorithm in self.algorithms}
            frames = {algorithm:None for algorithm in self.algorithms}
            # First handle the vanilla relaxed_ik values
            joints['relaxed'] =dpa_msg.angles_relaxed
            joints['lively'] = dpa_msg.angles_lively
            joints['joint_perlin'] = [clamp(angle,self.joint_limits[i][0],self.joint_limits[i][1]) for i,angle in enumerate(dpa_msg.angles_perlin)]
            #joints['joint_random_normal'] = [clamp(angle+self.last_random[i],self.joint_limits[i][0],self.joint_limits[i][1]) for i,angle in enumerate(dpa_msg.angles_relaxed)]
            #joints['joint_random_normal_ewma'] = [clamp(angle+self.last_random_ewma[i],self.joint_limits[i][0],self.joint_limits[i][1]) for i,angle in enumerate(dpa_msg.angles_relaxed)]
            #rospy.loginfo('writing js')
            for algorithm in self.algorithms:
                frames[algorithm] = self.robot.getFrames(joints[algorithm])
                if algorithm == 'relaxed':
                    collision[algorithm] = dpa_msg.collision_relaxed#self.collision_graph.get_collision_score(frames[algorithm]) > 5 #self.bullet_sim.check_collision(joints[algorithm])
                elif algorithm == 'lively':
                    collision[algorithm] = dpa_msg.collision_lively
                elif algorithm == 'joint_perlin':
                    collision[algorithm] = dpa_msg.collision_perlin
                info = [time,algorithm,collision[algorithm],dpa_msg.eval_type,dpa_msg.i] + list(joints[algorithm])
                self.js_file_writer.write(",".join([str(d) for d in info])+"\n")
            #rospy.loginfo('writing pose goal')
            # Transfer the poses from the actual goal
            for i in range(self.n_arms):
                pos = dpa_msg.ee_poses[i].position
                ori = dpa_msg.ee_poses[i].orientation
                info = [time,'goal',None,dpa_msg.eval_type,dpa_msg.i,i,pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w]
                self.pose_file_writer.write(",".join([str(d) for d in info])+"\n")
            #rospy.loginfo('writing ideal')
            # Transfer the "ideal" noise
            for i in range(self.n_arms):
                pos = dpa_msg.ideal_noise[i].position
                ori = dpa_msg.ideal_noise[i].orientation
                info = [time,'ideal',None,dpa_msg.eval_type,dpa_msg.i,i,pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w]
                self.pose_file_writer.write(",".join([str(d) for d in info])+"\n")
            #rospy.loginfo('writing algorithm')
            # Solve the positions that result for relaxed_ik and lively_k
            for algorithm in self.algorithms:
                poses = js_to_poses(self.robot,joints[algorithm])
                for i,pose in enumerate(poses):
                    info = [time,algorithm,collision[algorithm],dpa_msg.eval_type,dpa_msg.i,i,pose.position.x,pose.position.y,pose.position.z,pose.quaternion.x,pose.quaternion.y,pose.quaternion.z,pose.quaternion.w]
                    self.pose_file_writer.write(",".join([str(d) for d in info])+"\n")
            #rospy.loginfo('finished cb')

    def run(self):
        rostime = rospy.get_time()
        time = rostime - self.start_time
        #print("Time {0}".format(time))
        publish_initial = False

        if self.buffering:
            buffer_time = rostime - self.buffer_start_time
            #print("Buffer Time {0}".format(buffer_time))
            ProgressBar(buffer_time, self.buffer_time, prefix = "Buffer", suffix = '', decimals = 1, length = 100, fill = '=')
            if buffer_time > self.buffer_time:
                # Buffer is over, so begin the next test
                self.buffering = False
                print('')
                rospy.loginfo("Buffer Finished. Beginning Next Test...")
                self.tests[0].start()
            else:
                # Publish the start point of the next test
                publish_initial = True


        if publish_initial:
            poses,dc,bias,normal_weights,lively_weights = self.tests[0].initial
            i = self.tests[0].i
            test_name = "buffer"
        elif self.finished:
            pass
        else:
            poses,dc,bias,normal_weights,lively_weights = self.tests[0].command
            test_name = self.tests[0].name
            i = self.tests[0].i

        if self.finished or not self.tests[0].running and self.tests[0].started:
            if not self.finished:
                print('')
            if not self.finished:
                rospy.loginfo("Test Finished. Clearing Test...")
                self.tests.pop(0)
            if len(self.tests) == 0:
                if not self.finished:
                    rospy.loginfo("No more tests to run. Exiting...")
                poses = js_to_poses(self.robot,INITIAL_JOINTS[self.robot_name])
                i = 0
                dc = DC_DEFAULTS[self.robot_name]['lively']
                bias = BIAS[self.robot_name]['unbiased']
                normal_weights = WEIGHT_DEFAULTS[self.robot_name]['normal']
                lively_weights = WEIGHT_DEFAULTS[self.robot_name]['lively']
                test_name = 'exit'
                self.finished = True
            else:
                rospy.loginfo("More tests to run. Starting Buffer...")
                self.buffering = True
                self.buffer_start_time = rostime
                poses,dc,bias,normal_weights,lively_weights = self.tests[0].initial
                if self.tests[0].in_buffer:
                    test_name = 'buffer'
                else:
                    test_name = self.tests[0].name
                i = self.tests[0].i

        debug_goal = DebugGoals()
        debug_goal.header.seq =self.seq
        debug_goal.header.stamp = rospy.get_rostime()
        #rospy.loginfo('sending {0}'.format(test_name))
        debug_goal.eval_type = test_name
        debug_goal.i = i
        for pose in poses:
            debug_goal.ee_poses.append(pose.ros_pose)
        debug_goal.dc_values = dc
        debug_goal.bias = bias.ros_point
        debug_goal.normal_weights = normal_weights
        debug_goal.lively_weights = lively_weights

        self.goal_pub.publish(debug_goal)
        self.seq += 1

if __name__ == '__main__':
    #   rospy.sleep(180)
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
    evaluator = Eval(path_to_src,args.file_root,buffer_time=60)
    initialized = True

    while not rospy.get_param("ready"):
        rospy.sleep(1)

    rate = rospy.Rate(60)
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
