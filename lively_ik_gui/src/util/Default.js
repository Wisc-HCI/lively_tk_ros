
const defaultConfig = {
  axis_types:[],
  ee_fixed_joints:[],
  base_link_motion_bounds:[[0,0],[0,0],[0,0]],
  static_environment:{
    cuboids:[],
    spheres:[],
    pcs:[]
  },
  fixed_frame:'',
  goals:[],
  joint_limits:[],
  joint_names:[],
  joint_ordering:[],
  joint_types:[],
  mode_control:'absolute',
  mode_environment:'ECAA',
  nn_jointpoint:[],
  nn_main:[],
  objectives:[],
  states:[],
  robot_link_radius:0.05,
  rot_offsets:[],
  starting_config:[],
  urdf:'<?xml version="1.0" ?><robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"/><joint name="default_joint" type="fixed"><parent link="base_link" /><child link="default_link" /><origin xyz="0 0 0" rpy="0 0 0" /></joint><link name="default_link"/></robot>',
  velocity_limits:[],
  disp_offsets:[],
  displacements:[]
}

const defaultMeta = {
  valid_urdf: false,
  valid_robot: false,
  valid_nn: false,
  valid_config: false,
  valid_solver: false,
  displayed_state:[],
  links: [],
  dynamic_joints: [],
  fixed_joints: [],
  selected:null
}

const defaultObjectives = {
  joint_limits:{tag: 'Joint Limits', variant: 'joint_limits'},
  nn_collision:{tag: 'Self-Collision', variant: 'nn_collision'},
  env_collision:{tag: 'Dynamic Collision', variant: 'env_collision'},
  min_velocity:{tag: 'Minimize Velocity', variant: 'min_velocity'},
  min_acceleration:{tag: 'Minimize Acceleration', variant: 'min_acceleration'},
  min_jerk:{tag: 'Minimize Jerk', variant: 'min_jerk'},
  ee_position_match:{tag: 'EE Position Control', index: 0, variant: 'ee_position_match'},
  ee_orientation_match:{tag: 'EE Rotation Control', index: 0, variant: 'ee_orientation_match'},
  ee_position_mirroring:{tag: 'Mirror Arm Positions', index: 0, secondary_index: 1, variant: 'ee_position_mirroring'},
  ee_orientation_mirroring:{tag: 'Mirror Arm Orientations', index: 0, secondary_index: 1, variant: 'ee_orientation_mirroring'},
  ee_position_bounding:{tag: 'Arm Position Bounding', index: 0, variant: 'ee_position_bounding'},
  ee_orientation_bounding:{tag: 'Arm Orientation Bounding', index: 0, secondary_index: 1, variant: 'ee_orientation_bounding'},
  joint_mirroring:{tag: 'Mirror Joints', index: 0, secondary_index: 1, variant: 'joint_mirroring'},
  joint_match:{tag: 'Joint Control', index: 0, variant: 'joint_match'},
  ee_position_liveliness:{tag: 'Arm Position Liveliness', frequency:  5, index: 0, scale: 0.15, variant: 'ee_position_liveliness'},
  ee_orientation_liveliness:{tag: 'Arm Rotation Liveliness', frequency:  5, index: 0, scale: 1.0, variant: 'ee_orientation_liveliness'},
  joint_liveliness:{tag: 'Joint Liveliness', frequency: 5, index: 0, scale: 1.0, variant: 'joint_liveliness'},
  base_link_position_liveliness:{tag: 'Base Liveliness', frequency: 5, scale: 1.0, variant: 'base_link_position_liveliness'}
}

const defaultGoals = {
  joint_limits:{weight: 2.0},
  nn_collision:{weight: 5.0},
  env_collision:{weight: 5.0},
  min_velocity:{weight: 1},
  min_acceleration:{weight: 1},
  min_jerk:{weight: 0.1},
  ee_position_match:{vector: [0.0,0.0,0.0],weight: 30},
  ee_orientation_match:{quaternion: [1.0,0.0,0.0,0.0],weight: 25},
  ee_position_mirroring:{weight:50},
  ee_orientation_mirroring:{weight:50},
  ee_position_bounding:{weight:50},
  ee_orientation_bounding:{weight:50},
  joint_mirroring:{weight:50},
  joint_match:{weight:20},
  ee_position_liveliness:{weight:10},
  ee_orientation_liveliness:{weight:10},
  joint_liveliness:{weight:10},
  base_link_position_liveliness:{weight:10}
}

const defaultObjectiveNames = {
  joint_limits:'Joint Limits',
  nn_collision:'Static Collision',
  env_collision:'Dynamic Collision',
  min_velocity:'Velocity Minimization',
  min_acceleration:'Acceleration Minimization',
  min_jerk:'Jerk Minimization',
  ee_position_match:'Position Control',
  ee_orientation_match:'Rotation Control',
  ee_position_mirroring:'Position Mirroring',
  ee_orientation_mirroring:'Rotation Mirroring',
  ee_position_bounding:'Position Bounding',
  ee_orientation_bounding:'Rotation Bounding',
  joint_mirroring:'Joint Mirroring',
  joint_match:'Joint Control',
  ee_position_liveliness:'Position Liveliness',
  ee_orientation_liveliness:'Rotation Liveliness',
  joint_liveliness:'Joint Liveliness',
  base_link_position_liveliness:'Root Liveliness'
}

export {defaultConfig, defaultMeta, defaultObjectives, defaultGoals, defaultObjectiveNames}
