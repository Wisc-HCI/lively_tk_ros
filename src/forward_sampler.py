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
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj
from RelaxedIK.Utils import tf_fast as Tf
from wisc_msgs.msg import EEPoseGoals, JointAngles
from wisc_tools.structures import Pose
import rospy
import roslaunch
import os
import tf
import numpy as np
from argparse import ArgumentParser


class Monitor(object):
    def __init__(self,path_to_src,mode="publish",root="./noise"):
        self.fixed_frame = rospy.get_param('fixed_frame')
        self.joint_names = rospy.get_param('joint_names')
        self.joint_ordering = rospy.get_param('joint_ordering')

        if mode == "publish":
            self.collect_mode = False
            self.publish_mode = True
        elif mode == "collect":
            self.collect_mode = True
            self.publish_mode = False
            # Clear the current contents if they exist
            with open(root+"_js.csv","w") as js_file:
                js_file.write("time,"+",".join(self.joint_ordering))
            with open(root+"_goals.csv","w") as goal_file:
                js_file.write("time,"+",{0}_orig,{0}_noise")
            self.js_file_writer = open(root+"_js.csv",mode='a')
            self.goal_file_writer = open(root+"_goals.csv",mode='a')
        else:
            self.collect_mode = False
            self.publish_mode = False

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
        self.js_sub = rospy.Subscriber('/joint_states',JointState, self.js_sub_cb, queue_size=5)
        if self.publish_mode:
            self.goal_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals',EEPoseGoals, queue_size=5)
        self.tf_pub = tf.TransformBroadcaster()


    def js_sub_cb(self,js_msg):

        # Read the JointState messages,
        # and Use forward finematics to deduce
        # the pose goals
        state = [0]*len(self.joint_ordering)
        print(js_msg.position)
        if self.collect:

        for i,angle in enumerate(js_msg.position):
            name = js_msg.name[i]
            joint_index = self.joint_ordering.index(name)
            if joint_index > 0:
                state[joint_index] = angle
        frames = self.robot.getFrames(state)
        pairs = []
        ee_pose_goals = EEPoseGoals()
        for f in frames:
            position = f[0][-1]
            new_mat = np.zeros((4, 4))
            new_mat[0:3, 0:3] = f[1][-1]
            new_mat[3, 3] = 1
            rotation = Tf.euler_from_matrix(new_mat, 'szxy')
            info = {'position':{'x':position[0],'y':position[1],'z':position[2]},'rotation':{'r':rotation[0],'p':rotation[1],'y':rotation[2]}}
            pose = Pose.from_eulerpose_dict(info).ros_pose
            ee_pose_goals.ee_poses.append(pose)
            pairs.append(info)

        self.goal_pub.publish(ee_pose_goals)

        print("")
        print(",\n".join([str(pair) for pair in pairs]))

        # Update transform
        self.tf_pub.sendTransform((0,0,0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             'common_world',
                             self.fixed_frame)


if __name__ == '__main__':
    rospy.init_node('forward_monitor')

    parser = ArgumentParser(description='Forward Monitor')
    parser.add_argument("--file_root", dest="file_root", default="./noise", help="Specify the root name for files")
    args = parser.parse_known_args()[0]

    path_to_src = os.path.dirname(__file__)

    try:
        urdf_file_name = rospy.get_param('urdf_file_name')
        urdf_file = open(path_to_src + '/RelaxedIK/urdfs/' + urdf_file_name, 'r')
        urdf_string = urdf_file.read()
        rospy.set_param('robot_description', urdf_string)

        monitor = Monitor(path_to_src)
    except:
        rospy.logerr('Could not retrieve/apply required parameters!')

    rospy.sleep(0.2)

    while not rospy.is_shutdown():
        rospy.spin()
