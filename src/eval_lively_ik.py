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
from wisc_tools.convenience import ProgressBar

import rospy
import itertools
import os
import tf
import numpy as np
import random
from argparse import ArgumentParser
import math

JOINT_SCALES = {'elbow_joint': 0.017351484443726466,
                'shoulder_lift_joint': 0.0143959270733612,
                'shoulder_pan_joint': 0.016634272938059663,
                'wrist_1_joint': 0.029213377278518766,
                'wrist_2_joint': 0.02671317755209775,
                'wrist_3_joint': 0.0358804628491595}

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

    return PoseTrajectory([{'time': times[i], 'pose': poses[i]} for i in range(len(poses))],kind='cubic'), last_time

class Test(object):
    def __init__(self,name,i,robot,joint_limits):
        self.name = name
        self.i = i
        self.robot = robot
        self.joint_limits = joint_limits
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
        dc = [0,0,0,0,0,0]
        bias = Position(1,1,1)
        normal_weights = [50.0, 40.0,  0.0,  0.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        lively_weights = [25.0, 20.0, 25.0, 20.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        return UR3E_INITIAL_POSE,dc,bias,normal_weights,lively_weights

class ContinuousTest(Test):
    def __init__(self,i,robot,joint_limits,num_poses):
        super(ContinuousTest,self).__init__('Continuous',i,robot,joint_limits)
        self.num_poses = num_poses
        self.pose_trajectory, self.last_time = generate_pose_trajectory(robot,joint_limits,self.num_poses)
        self.start_time = 0

    def start(self):
        super(ContinuousTest,self).start()
        rospy.loginfo("Executing Continuous Trajectory Test {0} with {1} waypoints for {2} seconds.".format(self.i,len(self.pose_trajectory.wps),self.last_time))
        self.start_time = rospy.get_time()

    @property
    def initial(self):
        pose = self.pose_trajectory[0]
        dc = [0,0,0,0,0,0]
        bias = Position(1,1,1)
        normal_weights = [50.0, 40.0,  0.0,  0.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        lively_weights = [25.0, 20.0, 25.0, 20.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        return pose,dc,bias,normal_weights,lively_weights

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        if time > self.last_time:
            self.running = False
        ProgressBar(time, self.last_time, prefix = self.name, suffix = '', decimals = 1, length = 100, fill = '=')
        pose = self.pose_trajectory[time]
        dc = [0,0,0,0,0,0]
        bias = Position(1,1,1)
        normal_weights = [50.0, 40.0,  0.0,  0.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        lively_weights = [25.0, 20.0, 25.0, 20.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        return pose,dc,bias,normal_weights,lively_weights

class StaticDescriptiveTest(Test):
    def __init__(self,i,robot,joint_limits,duration):
        super(StaticDescriptiveTest,self).__init__('StaticDescriptive',i,robot,joint_limits)
        self.pose = generate_goal_pose(self.robot,self.joint_limits)
        self.last_time = duration
        self.start_time = 0
        self.in_buffer = True

    def start(self):
        super(StaticDescriptiveTest,self).start()
        rospy.loginfo("Executing Static Descriptive Test {0} for {1} seconds.".format(self.i,self.last_time))
        self.start_time = rospy.get_time()

    @property
    def initial(self):
        pose = self.pose
        dc = [0,0,0,0,0,0]
        bias = Position(1,1,1)
        normal_weights = [50.0, 40.0,  0.0,  0.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        lively_weights = [25.0, 20.0, 25.0, 20.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        return pose,dc,bias,normal_weights,lively_weights

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        ProgressBar(time, self.last_time, prefix = self.name, suffix = '', decimals = 1, length = 100, fill = '=')
        if time > self.last_time:
            self.running = False
        dc = [0,0,0,0,0,0]
        bias = Position(1,1,1)
        normal_weights = [50.0, 40.0,  0.0,  0.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        lively_weights = [25.0, 20.0, 25.0, 20.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        return self.pose,dc,bias,normal_weights,lively_weights

class DynamicFrequencyTest(Test):
    def __init__(self,i,robot,joint_limits,duration):
        super(DynamicFrequencyTest,self).__init__('DynamicFrequency',i,robot,joint_limits)
        self.pose = generate_goal_pose(self.robot,self.joint_limits)
        self.last_time = duration
        self.start_time = 0
        self.in_buffer = True

    def start(self):
        super(StaticDescriptiveTest,self).start()
        rospy.loginfo("Executing Dynamic Frequency Test {0} for {1} seconds.".format(self.i,self.last_time))
        self.start_time = rospy.get_time()

    @property
    def initial(self):
        pose = self.pose
        dc = [0,0,0,0,0,0]
        bias = Position(1,1,1)
        normal_weights = [50.0, 40.0,  0.0,  0.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        lively_weights = [25.0, 20.0, 25.0, 20.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        return pose,dc,bias,normal_weights,lively_weights

    @property
    def command(self):
        time = rospy.get_time() - self.start_time
        ProgressBar(time, self.last_time, prefix = self.name, suffix = '', decimals = 1, length = 100, fill = '=')
        if time > self.last_time:
            self.running = False
        dc = [0,0,0,0,0,0]
        bias = Position(1,1,1)
        normal_weights = [50.0, 40.0,  0.0,  0.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        lively_weights = [25.0, 20.0, 25.0, 20.0, 5.0, 3.0, 0.2, 1.0, 2.0]
        return self.pose,dc,bias,normal_weights,lively_weights

class Eval(object):
    def __init__(self,path_to_src,root="./noise", buffer_time=40):
        self.fixed_frame = rospy.get_param('fixed_frame')
        self.ee_fixed_joints = rospy.get_param('ee_fixed_joints')
        self.starting_config = rospy.get_param('starting_config')
        self.joint_names = rospy.get_param('joint_names')
        self.joint_ordering = rospy.get_param('joint_ordering')
        self.joint_limits = rospy.get_param('joint_limits')
        self.n_joints = len(self.joint_ordering)
        self.n_arms = len(self.ee_fixed_joints)
        self.joint_scaling = [JOINT_SCALES[joint] for joint in self.joint_ordering]
        self.joint_seeds = [random.randint(0,10240) for joint in self.joint_ordering]
        self.last_random = [0]*self.n_joints
        self.last_random_ewma = [0]*self.n_joints
        self.ewma_filter = EMA_filter([0]*self.n_joints,.1,75)
        self.algorithms = ['relaxed','lively','joint_perlin','joint_random_normal_ewma']
        self.last_time = 0
        self.running = False
        self.buffering = False
        self.buffer_time = buffer_time
        self.buffer_start_time = 0
        self.seq = 0

        # Clear the current contents if they exist
        with open(root+"_joints.csv","w") as js_file:
            header = ['time','algorithm','collision','eval','i']+self.joint_ordering
            # for group in ['relaxed','lively','joint_perlin','joint_random_normal','joint_random_normal_ewma']:
            #     header.extend([j+"_"+group for j in self.joint_ordering])
            js_file.write(",".join(header)+"\n")
        with open(root+"_poses.csv","w") as pose_file:
            values = ['x','y','z','qx','qy','qz','qw']
            header = ['time','algorithm','collision','eval','i']
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
            # ContinuousTest(self.robot,self.joint_limits,10),
            StaticDescriptiveTest(0,self.robot,self.joint_limits,300),
            # StaticDescriptiveTest(self.robot,self.joint_limits,300),
            # StaticDescriptiveTest(self.robot,self.joint_limits,300),
            # StaticDescriptiveTest(self.robot,self.joint_limits,300),
            # StaticDescriptiveTest(self.robot,self.joint_limits,300),
        ]

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
        # rospy.loginfo("Got Update!")
        if dpa_msg.eval_type == 'exit' or not self.running:
            self.running = False
        elif dpa_msg.eval_type == 'buffer':
            pass
        elif dpa_msg.eval_type != 'null':
            time = dpa_msg.header.stamp.to_sec()

            # Read the DebugPoseGoal messages,
            # and Use forward finematics to deduce
            # the poses that would result
            joints = self.n_joints*5*[0]
            perlin_noise = [(noise1D(time,self.joint_seeds[i],4))*(self.joint_scaling[i]/11.81) for i in range(self.n_joints)]
            normal_delta = [random.normalvariate(-1*self.last_random[i]*.01, self.joint_scaling[i]) for i in range(self.n_joints)]
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
                self.js_file_writer.write(",".join([str(time),algorithm,str(collision[algorithm]),dpa_msg.eval_type,str(dpa_msg.i)]+\
                                                   [str(j) for j in joints[algorithm]])+"\n")

            # Transfer the poses from the actual goal
            for i in range(self.n_arms):
                pos = dpa_msg.ee_poses[i].position
                ori = dpa_msg.ee_poses[i].orientation
                info = [time,'goal',None,dpa_msg.eval_type,dpa_msg.i,pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w]
            self.pose_file_writer.write(",".join([str(d) for d in info])+"\n")

            # Transfer the "ideal" noise
            for i in range(self.n_arms):
                pos = dpa_msg.ideal_noise[i].position
                ori = dpa_msg.ideal_noise[i].orientation
                info = [time,'ideal',None,dpa_msg.eval_type,dpa_msg.i,pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w]
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
                    info = [time,algorithm,collision[algorithm],dpa_msg.eval_type,dpa_msg.i,pos[0],pos[1],pos[2],ori.x,ori.y,ori.z,ori.w]
                    self.pose_file_writer.write(",".join([str(d) for d in info])+"\n")

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
            pose,dc,bias,normal_weights,lively_weights = self.tests[0].initial
            i = self.tests[0].i
            test_name = "buffer"
        else:
            pose,dc,bias,normal_weights,lively_weights = self.tests[0].command
            test_name = self.tests[0].name
            i = self.tests[0].i

        if not self.tests[0].running and self.tests[0].started:
            print('')
            rospy.loginfo("Test Finished. Clearing Test...")
            self.tests.pop(0)
            if len(self.tests) == 0:
                rospy.loginfo("No more tests to run. Exiting...")
                self.running = False
                pose = UR3E_INITIAL_POSE
                i = 0
                dc = [0,0,0,0,0,0]
                bias = Position(1,1,1)
                normal_weights = [50.0, 40.0,  0.0,  0.0, 5.0, 3.0, 0.2, 1.0, 2.0]
                lively_weights = [25.0, 20.0, 25.0, 20.0, 5.0, 3.0, 0.2, 1.0, 2.0]
                test_name = 'exit'
            else:
                rospy.loginfo("More tests to run. Starting Buffer...")
                self.buffering = True
                self.buffer_start_time = rostime
                pose,dc,bias,normal_weights,lively_weights = self.tests[0].initial
                if self.tests[0].in_buffer:
                    test_name = 'buffer'
                else:
                    test_name = self.tests[0].name
                i = self.tests[0].i

        debug_goal = DebugGoals()
        debug_goal.header.seq =self.seq
        debug_goal.header.stamp = rospy.get_rostime()
        debug_goal.eval_type = test_name
        debug_goal.i = i
        debug_goal.ee_poses.append(pose.ros_pose)
        debug_goal.dc_values = dc
        debug_goal.bias = bias.ros_point
        debug_goal.normal_weights = normal_weights
        debug_goal.lively_weights = lively_weights

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
