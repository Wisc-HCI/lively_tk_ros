DEFAULT_CONFIG = {
  'axis_types':[],
  'ee_fixed_joints':[],
  'base_link_motion_bounds':[[0,0],[0,0],[0,0]],
  'static_environment':{
    'cuboids':[],
    'spheres':[],
    'pcs':[]
  },
  'fixed_frame':'base_link',
  'modes':[{'name':'default', 'weights':[1.0, 2.0, 5.0]}],
  'goals':[{'name':'default', 'values':[{}, {}, {}]}],
  'joint_limits':[],
  'joint_names':[],
  'joint_ordering':[],
  'joint_types':[],
  'mode_control':'absolute',
  'mode_environment':'ECAA',
  'nn_jointpoint':{'intercepts':[],'coefs':[],'split_point':0},
  'nn_main':{'intercepts':[],'coefs':[],'split_point':0},
  'objectives':[
              {'tag': 'General Smoothness', 'variant': 'macro_smoothness', 'indices': []},
              {'tag': 'Joint Limits', 'variant': 'joint_limits', 'indices': []},
              {'tag': 'Self-Collision', 'variant': 'nn_collision', 'indices': []}
              ],
  'states':[],
  'robot_link_radius':0.05,
  'rot_offsets':[],
  'starting_config':[[0,0,0],[]],
  'urdf':'<?xml version="1.0" ?><robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"/><joint name="default_joint" type="fixed"><parent link="base_link" /><child link="default_link" /><origin xyz="0 0 0" rpy="0 0 0" /></joint><link name="default_link"/></robot>',
  'velocity_limits':[],
  'disp_offsets':[],
  'displacements':[]
}

DEFAULT_WEIGHTS = {
  'joint_limits':1.0,
  'nn_collision':5.0,
  'env_collision':5.0,
  'min_velocity':7.0,
  'min_acceleration':2.0,
  'min_jerk':1.0,
  'position_match':30.0,
  'orientation_match':25.0,
  'position_mirroring':50.0,
  'orientation_mirroring':30.0,
  'position_bounding':50.0,
  'orientation_bounding':30.0,
  'joint_mirroring':50.0,
  'joint_match':20.0,
  'position_liveliness':10.0,
  'orientation_liveliness':10.0,
  'joint_liveliness':10.0,
  'relative_motion_liveliness':10.0,
  'gravity':1.0,
  'macro_smoothness':1.0,
  'base_link_position_liveliness':10.0
}
