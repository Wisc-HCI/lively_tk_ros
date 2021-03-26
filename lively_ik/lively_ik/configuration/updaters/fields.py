from lively_ik.configuration.updaters.basic import *
from lively_ik.configuration.updaters.collision import *
from lively_ik.configuration.updaters.behaviors import *
from lively_ik.configuration.updaters.meta import *
from lively_ik.configuration.updaters.validity import *
from lively_ik_core import CONFIG_FIELDS
from datetime import datetime

FIELDS = {
    # Rust-Based Fields
    'valid_solver':{
        'derivation':derive_valid_solver,
        'dependencies':{'solver'}
    },
    'solver':{
        'derivation':derive_solver,
        'dependencies':{'config'}
    },
    'valid_config':{
        'derivation':derive_valid_config,
        'dependencies':{'config','objectives','goals','modes'}
    },
    'config':{
        'derivation':derive_config,
        'dependencies':CONFIG_FIELDS.union({'valid_nn'})
    },
    # Behavior-Based Fields
    # By default, add simple non-robot specific objectives
    'modes':{
        'derivation':derive_modes,
        'dependencies':{'objectives'}
    },
    'goals':{
        'derivation':derive_goals,
        'dependencies':{'objectives'}
    },
    'objectives':{
        'derivation':derive_objectives,
        'dependencies':{'valid_nn','joint_names','joint_ordering','ee_fixed_joints','robot_tree'}
    },
    # Neural-Network Fields
    'valid_nn':{
        'derivation':derive_valid_nn,
        'dependencies':{'train_directive','nn_main','nn_jointpoint'}
    },
    'nn_out_ts':{
        'derivation':lambda config: datetime.now().timestamp() if 'nn_out_ts' not in config or config['train_directive'] else config['nn_out_ts'],
        'dependencies':{'train_directive','nn_main','nn_jointpoint'}
    },
    'nn_precursor_ts':{
        'derivation':lambda config: datetime.now().timestamp(),
        'dependencies':{'train_directive','states','static_environment','robot','robot_link_radius'}
    },
    'nn_utd':{
        'derivation':lambda config: datetime.fromtimestamp(config['nn_out_ts']) > datetime.fromtimestamp(config['nn_precursor_ts']),
        'dependencies':{'train_directive','nn_out_ts','nn_precursor_ts'}
    },
    'nn_jointpoint':{
        'derivation':derive_nn_jointpoint,
        'dependencies':{'train_directive','training_frames','training_scores','collision_graph','robot'}
    },
    'nn_main':{
        'derivation':derive_nn_main,
        'dependencies':{'train_directive','training_samples','training_scores','collision_graph','robot'}
    },
    'training_scores':{
        'derivation':derive_training_scores,
        'dependencies':{'train_directive','robot','training_samples','collision_graph'}
    },
    'training_frames':{
        'derivation':derive_training_frames,
        'dependencies':{'train_directive','robot','training_samples'}
    },
    'training_samples':{
        'derivation':derive_training_samples,
        'dependencies':{'train_directive','base_link_motion_bounds','robot','states'}
    },
    'collision_graph':{
        'derivation':derive_collision_graph,
        'dependencies':{'train_directive','robot','states'}
    },
    'states':{
        'derivation':lambda config: [],
        'dependencies':set([])
    },
    'static_environment':{
        'derivation':lambda config: {'cuboids':[],'spheres':[],'pcs':[]},
        'dependencies':set([])
    },
    'robot_link_radius':{
        'derivation':lambda config: 0.05,
        'dependencies':set([])
    },
    'train_directive':{
        'derivation':lambda config: False,
        'dependencies':set([])
    },
    # Robot-Derived Fields
    'starting_config':{
        'derivation':derive_starting_config,
        'dependencies':{'joint_limits'}
    },
    'axis_types':{
        'derivation':derive_axis_types,
        'dependencies':{'robot'}
    },
    'joint_limits':{
        'derivation':derive_joint_limits,
        'dependencies':{'robot'}
    },
    'joint_types':{
        'derivation':derive_joint_types,
        'dependencies':{'robot'}
    },
    'velocity_limits':{
        'derivation':derive_velocity_limits,
        'dependencies':{'robot'}
    },
    'rot_offsets':{
        'derivation':derive_rot_offsets,
        'dependencies':{'robot'}
    },
    'disp_offsets':{
        'derivation':derive_disp_offsets,
        'dependencies':{'robot'}
    },
    'displacements':{
        'derivation':derive_displacements,
        'dependencies':{'robot'}
    },
    # Robot-Based Fields
    'valid_robot':{
        'derivation':lambda config: config['robot'] != None,
        'dependencies':{'robot'}
    },
    'robot':{
        'derivation':derive_robot,
        'dependencies':{'valid_arms','joint_names','urdf','extra_joints',
                        'joint_ordering','fixed_frame','ee_fixed_joints'}
    },
    'valid_arms':{
        'derivation':derive_valid_arms,
        'dependencies':{'parsed_urdf','joint_names','ee_fixed_joints',
                        'joint_ordering'}
    },
    'extra_joints':{
        'derivation':derive_extra_joints,
        'dependencies':{'joint_ordering','joint_names','robot_tree'}
    },
    'ee_fixed_joints':{
        'derivation':derive_ee_fixed_joints,
        'dependencies':{'joint_names','robot_tree'}
    },
    'fixed_frame':{
        'derivation':derive_fixed_frame,
        'dependencies':{'robot_tree'}
    },
    'joint_ordering':{
        'derivation':derive_joint_ordering,
        'dependencies':{'joint_names'}
    },
    'joint_names':{
        'derivation':derive_joint_names,
        'dependencies':{'fixed_frame','robot_tree'}
    },
    # Simple URDF Fields
    'links':{
        'derivation':derive_links,
        'dependencies':{'parsed_urdf'}
    },
    'fixed_joints':{
        'derivation':derive_fixed_joints,
        'dependencies':{'parsed_urdf'}
    },
    'dynamic_joints':{
        'derivation':derive_dynamic_joints,
        'dependencies':{'parsed_urdf'}
    },
    'robot_tree':{
        'derivation':derive_robot_tree,
        'dependencies':{'parsed_urdf'}
    },
    'valid_urdf':{
        'derivation':lambda config: config['parsed_urdf'] != [],
        'dependencies':{'parsed_urdf'}
    },
    'parsed_urdf':{
        'derivation':derive_parsed_urdf,
        'dependencies':{'urdf'}
    },
    'urdf':{
        'derivation':lambda config:'<?xml version="1.0" ?><robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"/><joint name="default_joint" type="fixed"><parent link="base_link" /><child link="default_link" /><origin xyz="0 0 0" rpy="0 0 0" /></joint><link name="default_link"/></robot>',
        'dependencies':set([])
    },
    ## -- Misc Fields -- ##
    # How much forgivenes the robot has to move its base-link
    'base_link_motion_bounds':{
        'derivation':lambda config: [[0,0],[0,0],[0,0]],
        'dependencies':set([])
    },
    # Whether the robot moves with absolute or relative control
    'mode_control':{
        'derivation':lambda config: 'absolute',
        'dependencies':set([])
    },
    # The type of real-time collision avoidance used.
    'mode_environment':{
        'derivation':lambda config: 'ECAA',
        'dependencies':set([])
    },
    ## -- GUI Fields -- ##
    # The actual displayed state to the front-end, if manual control is chosen.
    'displayed_state':{
        'derivation':derive_displayed_state,
        'dependencies':{'selected','starting_config','states'}
    },
    # Whether the robot should be controlled with LivelyIK or direct joints.
    # Either 'manual' or 'solve'. This should be 'manual' while configuring
    # joints in training states, or the initial state (starting_config).
    'control':{
        'derivation':derive_control,
        'dependencies':{'valid_solver'}
    },
    # What item is displayed in the side panel/drawer. If None, it shows nothing.
    # If not None, it should always have a type and index. Current valid types are
    # 'starting_config','collision_state','objective','mode', and 'goal'.
    'selected':{
        'derivation':lambda config: None,
        'dependencies':set([])
    },
    # The idealized data of the markers that are shown. These are
    # compared to the current set and published by the interface node.
    'markers':{
        'derivation':derive_markers,
        'dependencies':{'collision_markers','goal_markers','gui_markers'}
    },
    'collision_markers':{
        'derivation':derive_collision_markers,
        'dependencies':{'static_environment','show_link_collision',
                        'joint_names','joint_poses'}
    },
    'goal_markers':{
        'derivation':derive_goal_markers,
        'dependencies':{'valid_config','target_goals','selected','robot_tree',
                        'objectives','fixed_frame','joint_poses'}
    },
    'gui_markers':{
        'derivation':lambda config: {},
        'dependencies':{}
    },
    # This can be enabled or disabled from the front-end, and should
    # cause cylinder markers to show up in the visualizer window.
    'show_link_collision':{
        'derivation':lambda config: False,
        'dependencies':set([])
    },
    # The index of the mode that is actively being shown
    'active_mode':{
        'derivation':derive_active_mode,
        'dependencies':{'selected'}
    },
    # The index of the goal that is actively being shown
    'active_goals':{
        'derivation':derive_active_goals,
        'dependencies':{'selected'}
    },
    # The current position and orientation (euler rotation and quaternion) of each joint.
    # Note, organized first by chain, and then by joint. Has n+1 joints in each chain,
    # since it includes the end effector fixed joint.
    'joint_poses':{
        'derivation':derive_joint_poses,
        'dependencies':{'robot','displayed_state'}
    },
    # The weights that should be used by the solver (other than changes due to interpolation on the solver end)
    # This is derived from 'active_mode', but can be set separately in the case of configuring a mode, during
    # which the shown target weights are not the ones present in the stored mode's specification
    'target_weights':{
        'derivation':derive_target_weights,
        'dependencies':{'modes','active_mode'}
    },
    # The goals that should be used by the solver (other than changes due to interpolation on the solver end)
    # This is derived from 'active_goals', but can be set separately in the case of configuring a goal, during
    # which the shown target goal are not the ones present in the stored goal's specification
    'target_goals':{
        'derivation':derive_target_goals,
        'dependencies':{'goals','active_goals'}
    },
}
