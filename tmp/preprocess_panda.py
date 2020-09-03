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
        'robot_name':'panda',
        'urdf_file_name':'panda.urdf',
        'joint_names':[['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']],
        'joint_ordering':['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7'],
        'ee_fixed_joints':['panda_joint8'],
        'starting_config':[0.82,0.34,-0.24,-2.32,0.27,2.69,0.10],
        'collision_file_name':'collision_panda.yaml',
        'fixed_frame':'panda_joint0',
    }

    container = RelaxedIKContainer(info,node,config_override=True,pre_config=True)

    robot = container.robot
    num_chains = robot.numChains

    axis_types = []
    for i in range(num_chains):
        arm_axes = []
        chain_len = len(robot.arms[i].axes)
        for j in range(chain_len):
            arm_axes.append(robot.arms[i].axes[j])
        axis_types.append(arm_axes)
    info['axis_types'] = axis_types
    info['velocity_limits'] = robot.velocity_limits
    info['joint_limits'] = [[limit[0],limit[1]] for limit in robot.bounds]

    displacements = []
    for i in range(num_chains):
        arm_displacements = []
        chain_len = len(robot.arms[i].displacements)
        for j in range(chain_len):
            d = robot.arms[i].displacements[j]
            arm_displacements.append([d[0], d[1], d[2]])
        displacements.append(arm_displacements)
    info['displacements'] = displacements


    disp_offsets = []
    for i in range(num_chains):
        d = robot.arms[i].dispOffset
        disp_offsets.append([d[0], d[1], d[2]])
    info['disp_offsets'] = disp_offsets

    rot_offsets = []
    for i in range(num_chains):
        arm_offsets = []
        chain_len = len(robot.arms[i].original_rotOffsets)
        for j in range(chain_len):
            d = robot.arms[i].original_rotOffsets[j]
            arm_offsets.append([d[0], d[1], d[2]])
        rot_offsets.append(arm_offsets)
    info['rot_offsets'] = rot_offsets

    joint_types = []
    for i in range(num_chains):
        arm_types = []
        chain_len = len(robot.arms[i].joint_types)
        for j in range(chain_len):
            arm_types.append(robot.arms[i].joint_types[j])
        joint_types.append(arm_types)
    info['joint_types'] = joint_types


    joint_state_define_func_file_name = info['robot_name'] + '_joint_state_define'
    info['joint_state_define_func_file'] = joint_state_define_func_file_name

    with open(lively_ik.BASE + joint_state_define_func_file_name, 'w') as js_file:
        js_file.write("""def joint_state_define(x):\n\treturn None""")

    with open(lively_ik.BASE + '/config/info_files/' + info_file_name, 'w') as info_file:
        info_file.write(yaml.dump(info))

    with open(lively_ik.SRC + joint_state_define_func_file_name, 'w') as js_file:
        js_file.write("""def joint_state_define(x):\n\treturn None""")

    with open(lively_ik.SRC + '/config/info_files/' + info_file_name, 'w') as info_file:
        info_file.write(yaml.dump(info))

    print('info file {} successfully created!'.format(info_file_name))
