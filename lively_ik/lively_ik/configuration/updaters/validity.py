def derive_valid_robot(config):
    return config['robot'] != None

def derive_valid_urdf(config):
    return config['parsed_urdf'] != None

def derive_valid_config(config):
    return config['config'] != None

def derive_valid_solver(config):
    return config['solver'] != None

def derive_valid_nn(config):
    return config['nn_main'] != {'intercepts':[],'coefs':[],'split_point':None} and config['nn_jointpoint'] != {'intercepts':[],'coefs':[],'split_point':None}

def derive_valid_robot_output(config):
    passes = True
    if len(config['joint_limits']) != len(config['velocity_limits']) or len(config['joint_limits']) != len(config['joint_ordering']):
        passes = False
    for limit in config['joint_limits']:
        try:
            if len(limit) != 2:
                passes = False
        except:
            # The limit is probably a float
            passes = False
    for field in ['axis_types','joint_types','displacements','disp_offsets','rot_offsets']:
        if len(config[field]) != len(config['joint_names']):
            # print("chain length failure with {0}".format(field))
            # print(config[field])
            passes = False
        if field in ['axis_types','disp_offsets','rot_offsets','joint_types','displacements']:
            for idx,info in enumerate(config[field]):
                if field in ['axis_types','joint_types','displacements'] and len(info) != len(config['joint_names'][idx]):
                    passes = False
                    # print("detail failure with {0}".format(field))
                    # print(config[field])
                elif field == 'disp_offsets' and len(info) != 3:
                    passes = False
                    # print("detail failure with {0}".format(field))
                    # print(config[field])
    return passes
