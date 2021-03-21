from lively_ik_core import *
from lively_ik.configuration.transformations import quaternion_from_euler
from lively_ik.configuration.default import DEFAULT_WEIGHTS

def derive_config(config):
    if not config['valid_nn']:
        return None
    return parse_config_data({key:config[key] for key in CONFIG_FIELDS})

def derive_solver(config):
    try:
        return LivelyIK(config['config'])
    except:
        return None

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

def derive_modes(config):
    modes = [{'name':'default','weights':[]}]
    for objective in config['objectives']:
        modes[0]['weights'].append(DEFAULT_WEIGHTS[objective['variant']])
    return modes

def derive_objectives(config):
    objectives = []
    if not config['valid_nn']:
        return objectives

    # Add the smoothness macro
    objectives.append({'tag': 'General Smoothness', 'variant': 'macro_smoothness', 'indices': []})

    # Add Safety behaviors
    objectives.append({'tag': 'Joint Limits', 'variant': 'joint_limits', 'indices': []})
    objectives.append({'tag': 'Self-Collision', 'variant': 'nn_collision', 'indices': []})

    # For each of the chains, add a position_match objective at the end
    for chain_idx, chain in enumerate(config['joint_names']):
        objectives.append({'tag': '{0} Position Control'.format(config['ee_fixed_joints'][chain_idx]),
                           'variant': 'position_match', 'indices': [chain_idx,len(chain)]})

    # For each mimic joint, add a joint_mirroring objectives
    for joint_idx,joint in enumerate(config['joint_ordering']):
        joint_info = config['robot_tree']['joints'][joint]
        if joint_info['mimic'] != None and config['joint_ordering'].index(joint_info['mimic']) > 0:
            objectives.append({'tag': 'Mimic Joint {0}->{1}'.format(joint,joint_info['mimic']),
                               'variant': 'joint_mirroring', 'indices': [joint_idx,config['joint_ordering'].index(joint_info['mimic'])]})

    return objectives
