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
from wisc_tools.structures import Pose, Quaternion
import rospy
import itertools
import os
import tf
import numpy as np
import random
from argparse import ArgumentParser

class Monitor(object):
    def __init__(self,path_to_src,root="./noise"):
        self.fixed_frame = rospy.get_param('fixed_frame')
        self.ee_fixed_joints = rospy.get_param('ee_fixed_joints')
        self.starting_config = rospy.get_param('starting_config')
        self.joint_names = rospy.get_param('joint_names')
        self.joint_ordering = rospy.get_param('joint_ordering')
        self.joint_limits = rospy.get_param('joint_limits')
        self.n_joints = len(self.joint_ordering)
        self.n_arms = len(self.ee_fixed_joints)
        self.joint_scaling = [.01*abs(val[1]-val[0]) for val in self.joint_limits]
        self.joint_seeds = [random.randint(0,10240) for joint in self.joint_ordering]
        self.last_random = [0]*self.n_joints
        self.last_random_ewma = [0]*self.n_joints
        self.ewma_filter = EMA_filter([0]*self.n_joints,.1,75)
        self.algorithms = ['relaxed','lively','joint_perlin','joint_random_normal_ewma']

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
        self.dpa_sub = rospy.Subscriber('/relaxed_ik/debug_pose_angles',
                                        DebugPoseAngles,
                                        self.dpa_sub_cb,
                                        queue_size=5)

    def dpa_sub_cb(self,dpa_msg):
        rospy.loginfo("Got Update!")
        time = dpa_msg.header.stamp.to_sec()

        # Read the DebugPoseGoal messages,
        # and Use forward finematics to deduce
        # the poses that would result
        joints = self.n_joints*5*[0]
        perlin_noise = [(noise1D(time/4,self.joint_seeds[i])-0.5)*self.joint_scaling[i] for i in range(self.n_joints)]
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
        goal_info = []
        for i in range(self.n_arms):
            pos = dpa_msg.ee_poses[i].position
            ori = dpa_msg.ee_poses[i].orientation
            info = [time,'goal',None,dpa_msg.eval_type,pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w]
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

        monitor = Monitor(path_to_src,args.file_root)
        initialized = True
    except Exception as e:
        rospy.logerr('Could not retrieve/apply required parameters!')
        rospy.logerr(str(e))

    while not rospy.is_shutdown():
        rospy.spin()

    try:
        monitor.js_file_writer.close()
        monitor.pose_file_writer.close()
    except:
        if initialized:
            rospy.logerr('Could not close files!')
