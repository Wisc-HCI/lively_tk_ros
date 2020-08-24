import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

BASE = get_package_share_directory('lively_ik')
SRC = str(Path(os.path.realpath(__file__)).parent.parent)

INFO_PARAMS = ['axis_types', 'collision_file_name',
               'collision_nn_file', 'disp_offsets',
               'displacements', 'ee_fixed_joints',
               'fixed_frame', 'fixed_frame_noise_scale',
               'fixed_frame_noise_frequency', 'joint_limits',
               'joint_names', 'joint_ordering',
               'joint_state_define_func_file', 'joint_types',
               'mode', 'objectives', 'path_to_src',
               'rot_offsets', 'starting_config', 'urdf_file_name',
               'velocity_limits', 'dc_joint_noise', 'dc_joint_weight',
               'ee_joint_noise', 'ee_position_weight',
               'ee_rotation_weight', 'match_objectives']
