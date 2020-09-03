#! /usr/bin/env python

######################################################################################################

# from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
#     joint_state_define, collision_file_name, fixed_frame
# from RelaxedIK.relaxedIK import RelaxedIK
# from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
# from sensor_msgs.msg import JointState
# import rospy
# import roslaunch
# import os
# import tf
# import math
# from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj
import rclpy
from rclpy.node import Node
from lively_ik.groove.relaxed_ik_container import RelaxedIKContainer

if __name__ == '__main__':
    rclpy.init()
    node = Node('preprocessing')

    info = {
        'urdf_file_name':'fetch.urdf',
        'joint_names':[["torso_lift_joint","shoulder_pan_joint","shoulder_lift_joint","upperarm_roll_joint","elbow_flex_joint","forearm_roll_joint","wrist_flex_joint","wrist_roll_joint"]],
        'joint_ordering':["torso_lift_joint","shoulder_pan_joint","shoulder_lift_joint","upperarm_roll_joint","elbow_flex_joint","forearm_roll_joint","wrist_flex_joint","wrist_roll_joint"],
        'ee_fixed_joints':["gripper_axis"],
        'starting_config':[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
        'collision_file_name':'collision_fetch.yaml',
        'collision_nn_file':'fetch_nn',
        'fixed_frame':'base_link',
    }

    container = RelaxedIKContainer(info,node,config_override=True,pre_config=False)
