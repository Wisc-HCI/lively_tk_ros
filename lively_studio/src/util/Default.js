
const defaultConfig = {
  axis_types:[],
  fixed_joints:[],
  base_link_motion_bounds:[[0,0],[0,0],[0,0]],
  static_environment:{
    cuboids:[],
    spheres:[],
    pcs:[]
  },
  fixed_frame:'base_link',
  modes:[{name:'default',weights:[1.0,2.0,5.0]}],
  goals:[{name:'default',values:[{},{},{}]}],
  joint_limits:[],
  joint_names:[],
  joint_ordering:[],
  joint_types:[],
  mode_control:'absolute',
  mode_environment:'ECAA',
  nn_jointpoint:[],
  nn_main:[],
  objectives:[{tag: 'Smoothness Macro', variant: 'macro_smoothness', indices: []},
              {tag: 'Joint Limits', variant: 'joint_limits', indices: []},
              {tag: 'Self-Collision', variant: 'nn_collision', indices: []}],
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
  nn_main_utd: false,
  nn_jointpoint_utd: false,
  displayed_state:[],
  highlights:[],
  control:'manual',
  show_link_collision:false,
  nn_progress:0,
  joint_poses:[],
  links: [],
  dynamic_joints: [],
  fixed_joints: [],
  selected:null,
  updating:false,
  active_goals:0,
  active_mode:0,
  target_weights:[],
  target_goals:[]
}

const defaultObjectives = {
  joint_limits:{tag: 'Joint Limits', variant: 'joint_limits', indices: []},
  macro_smoothness:{tag: 'Smoothness Macro', variant: 'macro_smoothness', indices: []},
  nn_collision:{tag: 'Self-Collision', variant: 'nn_collision', indices: []},
  env_collision:{tag: 'Dynamic Collision', variant: 'env_collision', indices: []},
  min_velocity:{tag: 'Minimize Velocity', variant: 'min_velocity', indices: []},
  min_acceleration:{tag: 'Minimize Acceleration', variant: 'min_acceleration', indices: []},
  min_jerk:{tag: 'Minimize Jerk', variant: 'min_jerk', indices: []},
  position_match:{tag: 'Position Control', variant: 'position_match', indices: [0,0]},
  orientation_match:{tag: 'Rotation Control', variant: 'orientation_match', indices: [0,0]},
  position_mirroring:{tag: 'Mirror Arm Positions', variant: 'position_mirroring', indices: [0,0,0,0]},
  orientation_mirroring:{tag: 'Mirror Arm Orientations', variant: 'orientation_mirroring', indices: [0,0,0,0]},
  position_bounding:{tag: 'Arm Position Bounding', variant: 'position_bounding', indices: [0,0]},
  orientation_bounding:{tag: 'Arm Orientation Bounding', variant: 'orientation_bounding', indices: [0,0]},
  joint_mirroring:{tag: 'Mirror Joints', variant: 'joint_mirroring', indices: [0,0]},
  joint_match:{tag: 'Joint Control', variant: 'joint_match', indices: [0]},
  position_liveliness:{tag: 'Arm Position Liveliness', frequency:  5, shape: [0.15,0.15,0.15], variant: 'position_liveliness', indices: [0,0]},
  orientation_liveliness:{tag: 'Arm Rotation Liveliness', frequency:  5, shape: [0.15,0.15,0.15], variant: 'orientation_liveliness', indices: [0,0]},
  joint_liveliness:{tag: 'Joint Liveliness', frequency: 5, scale: 1.0, variant: 'joint_liveliness', indices: [0]},
  relative_motion_liveliness:{tag: 'Lively Extension', frequency: 5, scale: 0.1, variant: 'relative_motion_liveliness', indices: [0,0,0,0]},
  base_link_position_liveliness:{tag: 'Base Liveliness', frequency: 5, shape: [0.15,0.15,0.15], variant: 'base_link_position_liveliness', indices: []},
  base_link_position_match:{tag: 'Base Position Control', variant: 'base_link_position_match', indices: []},
  gravity:{tag: 'Gravity', variant: 'gravity', indices: [0,0]},
  distance_match:{tag: 'Distance Control', variant:'distance_match', indices:[0,0,0,0]}

}

const objectiveMeta = {
  joint_limits:{
    goal:{},
    weight:2.0,
    name:'Joint Limits',
    weightLimits:[0,20]
  },
  nn_collision:{
    goal:{},
    weight:0.01,
    name:'Static Collision',
    weightLimits:[0,20]
  },
  env_collision:{
    goal:{},
    weight:5.0,
    name:'Dynamic Collision',
    weightLimits:[0,20]
  },
  min_velocity:{
    goal:{},
    weight:1.0,
    name:'Velocity Minimization',
    weightLimits:[0,20]
  },
  min_acceleration:{
    goal:{},
    weight:1.0,
    name:'Acceleration Minimization',
    weightLimits:[0,20]
  },
  min_jerk:{
    goal:{},
    weight:0.1,
    name:'Jerk Minimization',
    weightLimits:[0,20]
  },
  position_match:{
    goal:{vector: [0.0,0.0,0.0]},
    weight:30.0,
    name:'Position Control',
    weightLimits:[0,100]
  },
  orientation_match:{
    goal:{quaternion: [1.0,0.0,0.0,0.0]},
    weight:25.0,
    name:'Rotation Control',
    weightLimits:[0,100]
  },
  position_mirroring:{
    goal:{},
    weight:50.0,
    name:'Position Mirroring',
    weightLimits:[0,100]
  },
  orientation_mirroring:{
    goal:{},
    weight:30.0,
    name:'Rotation Mirroring',
    weightLimits:[0,100]
  },
  position_bounding:{
    goal:{},
    weight:50.0,
    name:'Position Bounding',
    weightLimits:[0,100]
  },
  orientation_bounding:{
    goal:{},
    weight:50.0,
    name:'Rotation Bounding',
    weightLimits:[0,100]
  },
  joint_mirroring:{
    goal:{},
    weight:50.0,
    name:'Joint Mirroring',
    weightLimits:[0,100]
  },
  joint_match:{
    goal:{scalar: 0},
    weight:20,
    name:'Joint Control',
    weightLimits:[0,50]
  },
  position_liveliness:{
    goal:{},
    weight:10.0,
    name:'Position Liveliness',
    weightLimits:[0,30]
  },
  orientation_liveliness:{
    goal:{},
    weight:8.0,
    name:'Rotation Liveliness',
    weightLimits:[0,30]
  },
  joint_liveliness:{
    goal:{},
    weight:10.0,
    name:'Joint Liveliness',
    weightLimits:[0,30]
  },
  relative_motion_liveliness:{
    goal:{},
    weight:10.0,
    name:'Relative Motion',
    weightLimits:[0,30]
  },
  base_link_position_liveliness:{
    goal:{vector: [0.0,0.0,0.0]},
    weight:15.0,
    name:'Robot Root Liveliness',
    weightLimits:[0,20]
  },
  base_link_position_match:{
    goal:{},
    weight:30.0,
    name:'Robot Root Position Control',
    weightLimits:[0,50]
  },
  gravity:{
    goal:{},
    weight:15.0,
    name:'Gravity',
    weightLimits:[0,30]
  },
  macro_smoothness:{
    goal:{},
    weight:2.0,
    name:'Smoothness Macro',
    weightLimits:[0,20]
  },
  distance_match:{
    goal:{scalar:0.1},
    weight:10.0,
    name:'Distance Control',
    weightLimits:[0,50]
  }
}

export {defaultConfig, defaultMeta, defaultObjectives, objectiveMeta}
