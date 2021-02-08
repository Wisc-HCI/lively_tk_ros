
let config = {
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

let meta = {
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

export {config, meta}
