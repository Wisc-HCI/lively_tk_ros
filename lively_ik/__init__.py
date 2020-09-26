import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory

BASE = get_package_share_directory('lively_ik')
SRC = str(Path(os.path.realpath(__file__)).parent.parent)

INFO_PARAMS = {'axis_types','boxes','capsules','cylinders','displacements',
               'disp_offsets','ee_fixed_joints','ellipsoids','fixed_frame',
               'fixed_frame_noise_scale','fixed_frame_noise_frequency',
               'joint_limits','joint_names','joint_ordering','joint_types',
               'mode','objectives','problem_states','robot_link_radius',
               'robot_name','rot_offsets','sample_states','spheres',
               'starting_config','training_states','urdf','velocity_limits'}

def get_configs():
    configs = {}
    config_files = [f for f in os.listdir(BASE+'/config/info_files') if os.path.isfile(os.path.join(BASE+'/config/info_files', f))]
    for config_file in config_files:
        with open(BASE+'/config/info_files/'+config_file) as io:
            config_data = yaml.safe_load(io)
        if INFO_PARAMS.issubset(set(config_data.keys())):
            configs[config_data['robot_name']] = config_data
    return configs
