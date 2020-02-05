#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 11/4/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''
######################################################################################################

import os
import rospy
from RelaxedIK.Utils.colors import bcolors
from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from start_here import info_file_name, urdf_file_name, fixed_frame, joint_names, joint_ordering, \
                       ee_fixed_joints, ee_joint_noise, fixed_frame_noise, starting_config, \
                       collision_file_name, joint_state_define, dc_joint_noise, dc_joint_weight, \
                       ee_position_weight, ee_rotation_weight, match_objectives
import inspect
import yaml

rospy.init_node('generate_info_file')

path_to_src = os.path.dirname(__file__)

if info_file_name == '':
    print bcolors.FAIL + 'info_file_name is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)

config = {}

if urdf_file_name == '':
    print bcolors.FAIL + 'urdf_file_name is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    config['urdf_file_name'] = urdf_file_name

if fixed_frame == '':
    print bcolors.FAIL + 'fixed_frame is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    config['fixed_frame'] = fixed_frame

if len(joint_names) == 0:
    print bcolors.FAIL + 'joint_names is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    config['joint_names'] = joint_names

if len(joint_ordering) == 0:
    print bcolors.FAIL + 'joint_ordering is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    config['joint_ordering'] = joint_ordering

if len(ee_fixed_joints) == 0:
    print bcolors.FAIL + 'ee_fixed_joints is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    config['ee_fixed_joints'] = ee_fixed_joints

# Lively features
config['ee_position_weight'] = ee_position_weight
config['ee_rotation_weight'] = ee_rotation_weight
config['ee_joint_noise'] = ee_joint_noise
config['dc_joint_noise'] = dc_joint_noise
config['dc_joint_weight'] = dc_joint_weight
config['fixed_frame_noise'] = fixed_frame_noise

# Match Objectives
config['match_objectives'] = match_objectives

if len(starting_config) == 0:
    print bcolors.FAIL + 'starting_config is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    config['starting_config'] = starting_config


if collision_file_name == '':
    print bcolors.FAIL + 'collision_file_name is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    config['collision_file_name'] = collision_file_name

robot_name_split = urdf_file_name.split('.')
robot_name = robot_name_split[0]

config['collision_nn_file'] = robot_name + '_nn'
config['path_to_src'] = path_to_src

# AUTO############################################################################################################################################
##################################################################################################################################################

vars = RelaxedIK_vars('', path_to_src + '/RelaxedIK/urdfs/' + urdf_file_name, joint_names, ee_fixed_joints, joint_ordering, pre_config=True, init_state=starting_config, path_to_src=path_to_src)


robot = vars.robot
num_chains = robot.numChains

axis_types = []
for i in range(num_chains):
    arm_axes = []
    chain_len = len(robot.arms[i].axes)
    for j in range(chain_len):
        arm_axes.append(robot.arms[i].axes[j])
    axis_types.append(arm_axes)
config['axis_types'] = axis_types
config['velocity_limits'] = robot.velocity_limits
config['joint_limits'] = [[limit[0],limit[1]] for limit in robot.bounds]

displacements = []
for i in range(num_chains):
    arm_displacements = []
    chain_len = len(robot.arms[i].displacements)
    for j in range(chain_len):
        d = robot.arms[i].displacements[j]
        arm_displacements.append([d[0], d[1], d[2]])
    displacements.append(arm_displacements)
config['displacements'] = displacements


disp_offsets = []
for i in range(num_chains):
    d = robot.arms[i].dispOffset
    disp_offsets.append([d[0], d[1], d[2]])
config['disp_offsets'] = disp_offsets

rot_offsets = []
for i in range(num_chains):
    arm_offsets = []
    chain_len = len(robot.arms[i].original_rotOffsets)
    for j in range(chain_len):
        d = robot.arms[i].original_rotOffsets[j]
        arm_offsets.append([d[0], d[1], d[2]])
    rot_offsets.append(arm_offsets)
config['rot_offsets'] = rot_offsets

joint_types = []
for i in range(num_chains):
    arm_types = []
    chain_len = len(robot.arms[i].joint_types)
    for j in range(chain_len):
        arm_types.append(robot.arms[i].joint_types[j])
    joint_types.append(arm_types)
config['joint_types'] = joint_types


joint_state_define_func_file_name = robot_name + '_joint_state_define'
fp = path_to_src + '/RelaxedIK/Config/joint_state_define_functions/' + joint_state_define_func_file_name
js_file = open(fp, 'w')
js_file.write(inspect.getsource(joint_state_define))
config['joint_state_define_func_file'] = joint_state_define_func_file_name

with open(path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name, 'w') as info_file:
    info_file.write(yaml.dump(config))

print bcolors.OKGREEN + 'info file {} successfully created!'.format(info_file_name) + bcolors.ENDC

'''
in_file = open(path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name, 'r')

import yaml

y = yaml.load(in_file)
'''
