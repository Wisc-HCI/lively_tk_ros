def derive_valid_arms(config):
    if config['parsed_urdf'] == []:
        return False
    if len(config['joint_names']) == len(config['ee_fixed_joints']) and len(config['joint_names']) > 0 and len(config['joint_ordering']) > 0:
        for chain in config['joint_names']:
            for joint in chain:
                if joint not in config['joint_ordering']:
                    return False
        result = True
    else:
        result = False
    return result

def derive_valid_config(config):
    if config['config'] == None:
        return False
    num_objectives = len(config['objectives'])
    for mode in config['modes']:
        mode_length = len(mode['weights'])
        if mode_length != num_objectives:
            return False
    for goal in config['goals']:
        goal_length = len(goal['values'])
        if goal_length != num_objectives:
            return False
    return True

def derive_valid_solver(config):
    return config['solver'] != None

def derive_valid_nn(config):
    return config['nn_main'] != {'intercepts':[],'coefs':[],'split_point':None} and config['nn_jointpoint'] != {'intercepts':[],'coefs':[],'split_point':None}
