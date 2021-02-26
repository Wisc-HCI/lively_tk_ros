from lively_ik_core import *
from lively_ik.configuration.transformations import quaternion_from_euler

def derive_config(config):
    return parse_config_data({key:config[key] for key in CONFIG_FIELDS})

def derive_solver(config):
    return LivelyIK(config['config'])

def derive_goals(config):
    goals = [{'name':'default','values':[]}]
    for objective in config['objectives']:
        if objective['variant'] == 'position_match':
            arm_idx = objective['indices'][0]
            jnt_idx = objective['indices'][1]
            position = config['joint_poses'][arm_idx][jnt_idx]['position']
            goals[0]['values'].append({'vector':[position['x'],position['y'],position['z']]})
        elif objective['variant'] == 'orientation_match':
            arm_idx = objective['indices'][0]
            jnt_idx = objective['indices'][1]
            rotation = config['joint_poses'][arm_idx][jnt_idx]['rotation']
            quat = list(quaternion_from_euler(rotation['r'],rotation['p'],rotation['y'],'szxy'))
            goals[0]['values'].append({'quaternion':quat})
        elif objective['variant'] == 'joint_match':
            jnt_idx = objective['indices'][0]
            goals[0]['values'].append({'scalar':config['starting_config'][jnt_idx]})
        else:
            goals[0]['values'].append({})
    return goals
