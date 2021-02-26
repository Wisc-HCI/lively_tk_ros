def derive_displayed_state(config):
    if config['selected'] != None:
        if config['selected']['type'] == 'starting_config':
            return config['starting_config']
        elif config['selected']['type'] == 'collision_state' and config['selected']['idx'] != None:
            return config['states'][idx]
    else:
        return config['starting_config']

def derive_control(config):
    if config['valid_solver']:
        control = 'solve'
    else:
        control = 'manual'
    return control

def derive_markers(config):
    markers = {}

    # Start with static environment
    for i,collision_cuboid in enumerate(config['static_environment']['cuboids']):
        marker_data = {
            'frame_id':collision_cuboid['coordinate_frame'],
            'pose':{'position':{'x':collision_cuboid['tx'],'y':collision_cuboid['ty'],'z':collision_cuboid['tz']},
                    'rotation':{'r':collision_cuboid['rx'],'p':collision_cuboid['ry'],'y':collision_cuboid['rz']}
                   },
            'scale':{'x':collision_cuboid['x_halflength']*2,'y':collision_cuboid['y_halflength']*2,'z':collision_cuboid['z_halflength']*2},
            'type':'cube',
            'color':{'r':0.83,'g':0.63,'b':0.04,'a':0.5}
        }
        markers['collision_cuboid_{0}'.format(i)] = marker_data

    for i,collision_sphere in enumerate(config['static_environment']['spheres']):
        marker_data = {
            'frame_id':collision_sphere['coordinate_frame'],
            'pose':{'position':{'x':collision_sphere['tx'],'y':collision_sphere['ty'],'z':collision_sphere['tz']},
                    'rotation':{'r':0,'p':0,'y':0}
                   },
            'scale':{'x':collision_sphere['radius']*2,'y':collision_sphere['radius']*2,'z':collision_sphere['radius']*2},
            'type':'sphere',
            'color':{'r':0.83,'g':0.63,'b':0.04,'a':0.5}
        }
        markers['collision_sphere_{0}'.format(i)] = marker_data

    # Add any highlighted joints or links
    for highlight in config['highlights']:
        if highlight['id'] in config['robot_tree']:
            robot_node = config['robot_tree'][highlight['id']]
            if robot_node['type'] == 'link':
                marker_data = {
                    'frame_id':highlight['id'],
                    'pose':{'position':{'x':0,'y':0,'z':0},
                            'rotation':{'r':0,'p':0,'y':0}
                           },
                    'scale':{'x':1,'y':1,'z':1},
                    'type':robot_node['model'],
                    'color':{'r':0.09,'g':0.56,'b':1.0,'a':0.5}
                }
                markers['highlight_{0}'.format(highlight['id'])] = marker_data
            elif robot_node['type'] == 'joint':
                marker_data = {
                    'frame_id':highlight['id'],
                    'pose':{'position':{'x':0,'y':0,'z':0},
                            'rotation':{'r':0,'p':0,'y':0}
                           },
                    'scale':{'x':0.2,'y':0.2,'z':0.2},
                    'type':'sphere',
                    'color':{'r':0.75,'g':0.2,'b':1.0,'a':0.5}
                }
                markers['highlight_{0}'.format(highlight['id'])] = marker_data

    if config['show_link_collision']:
        for joint in config['joint_ordering']:
            parent_link = config['robot_tree'][joint]['parent']
            # TODO: calculate midpoint and attach marker there.


    # Add lively previews, if selected
    if config['selected'] and config['selected']['type'] == 'objective':
        objective = config['objectives'][config['selected']['idx']]
        if objective['variant'] in ['position_liveliness','orientation_liveliness']:
            arm_index = int(objective['indices'][0])
            joint_index = int(objective['indices'][1])
            position = config['joint_poses'][arm_index][joint_index]['position']
            if objective['variant'] == 'position_liveliness':
                marker_data = {
                    'frame_id':config['fixed_frame'],
                    'pose':{'position':position,
                            'rotation':{'r':0,'p':0,'y':0}
                           },
                    'scale':{'x':objective['shape'][0],'y':objective['shape'][1],'z':objective['shape'][2]},
                    'type':'sphere',
                    'color':{'r':0.09,'g':0.56,'b':1.0,'a':0.5}
                }
                markers['objective_{0}'.format(objective['tag'])] = marker_data
            elif objective['variant'] == 'orientation_liveliness':
                # TODO: Show orientation in some way
                pass


    return markers

def derive_active_mode(config):
    if config['selected'] and config['selected']['type'] == 'mode':
        mode = config['modes'][config['selected']['idx']]['name']
    else:
        mode = 'default'
    return mode

def derive_active_goals(config):
    if config['selected'] and config['selected']['type'] == 'goal':
        goals = config['goals'][config['selected']['idx']]['name']
    else:
        goals = 'default'
    return goals

def derive_target_weights(config):
    mode = [mode for mode in config['modes'] if mode['name'] == config['active_mode']][0]
    return mode['weights']

def derive_target_goals(config):
    goal = [goal for goal in config['goals'] if goal['name'] == config['active_goals']][0]
    return goal['values']
