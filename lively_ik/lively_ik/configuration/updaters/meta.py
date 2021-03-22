import numpy as np

COLLISION_OBJECT_COLOR = {'r':0.83,'g':0.63,'b':0.04,'a':0.5}
GOAL_COLOR = {'r':0.09,'g':0.56,'b':1.0,'a':0.3}
HIGHLIGHT_COLOR = {'r':0.09,'g':1.0,'b':0.95,'a':0.5}

def derive_displayed_state(config):
    if config['selected'] != None:
        if config['selected']['type'] == 'starting_config':
            return config['starting_config']
        elif config['selected']['type'] == 'collision_state' and config['selected']['idx'] != None:
            return config['states'][idx]
        else:
            return config['starting_config']
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
    for marker_set in ['collision_markers','highlight_markers','goal_markers']:
        for name,info in config[marker_set].items():
            markers[name] = info
    return markers

def derive_collision_markers(config):
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
            'color':COLLISION_OBJECT_COLOR
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
            'color':COLLISION_OBJECT_COLOR
        }
        my_str = 'my_string_is_substituted_with_{0}_and_{1}'.format("hi","bye")
        markers['marker_key'] = {}
        markers['collision_sphere_{0}'.format(i)] = marker_data


    if config['show_link_collision']:
        for arm_idx,arm in enumerate(config['joint_names']):
            for joint_idx,joint in enumerate(arm):
                # Joint is the name of the joint, you need to find the name of the link
                if joint_idx+1 < len(arm):
                    joint_1_pose_info = config['joint_poses'][arm_idx][joint_idx]
                    joint_2_pose_info = config['joint_poses'][arm_idx][joint_idx+1]
                    mdpt = {'x':(joint_1_pose_info['position']['x']+joint_2_pose_info['position']['x'])/2-joint_1_pose_info['position']['x'],
                            'y':(joint_1_pose_info['position']['y']+joint_2_pose_info['position']['y'])/2-joint_1_pose_info['position']['y'],
                            'z':(joint_1_pose_info['position']['z']+joint_2_pose_info['position']['z'])/2-joint_1_pose_info['position']['z']}
                    link_name = config['robot_tree']['joints'][joint]['parent'] # Could be child
                    marker_data = {
                        #Change these values
                        'frame_id':link_name,
                        # Pose will be determined by how the coordinate frame is structured
                        # Rotation is probably the same as here
                        'pose':{'position':{'x':None,'y':None,'z':None},
                                'rotation':{'r':0,'p':0,'y':0}
                               },
                        # Figure out scale
                        # from http://wiki.ros.org/rviz/DisplayTypes/Marker
                        # scale.x is diameter in x direction, scale.y in y direction,
                        # by setting these to different values you get an ellipse
                        # instead of a circle. Use scale.z to specify the height.
                        'scale':{'x':collision_sphere['radius']*2,'y':collision_sphere['radius']*2,'z':collision_sphere['radius']*2},
                        'type':'cylinder',
                        'color':COLLISION_OBJECT_COLOR
                    }
                    markers['collision_{0}'.format(link)] = marker_data

    return markers

def derive_highlight_markers(config):
    markers = {}
    # Add any highlighted joints or links
    for highlight in config['highlights']:
        if highlight['id'] in config['robot_tree']['links']:
            robot_node = config['robot_tree']['links'][highlight['id']]
            marker_data = {
                'frame_id':highlight['id'],
                'pose':{'position':{'x':0,'y':0,'z':0},
                        'rotation':{'r':0,'p':0,'y':0}
                       },
                'scale':{'x':1,'y':1,'z':1},
                'type':robot_node['model'],
                'color':HIGHLIGHT_COLOR
            }
            markers['highlight_{0}'.format(highlight['id'])] = marker_data
        elif highlight['id'] in config['robot_tree']['joints']:
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

    return markers

def derive_goal_markers(config):
    markers = {}
    if config['valid_config']:
        # Add goals, and highlight them if they are selected by that objective
        for idx,goal_value in enumerate(config['target_goals']):
            if  config['selected'] and config['selected']['type'] == 'objective' and config['selected']['idx'] == idx:
                color = HIGHLIGHT_COLOR
            else:
                color = GOAL_COLOR
            # Get the objective information, since this is useful in some cases
            try:
                objective = config['objectives'][idx]
            except:
                objective = {'variant':None}

            # First check the goal value itself and discern the type.
            if 'vector' in goal_value.keys():
                position = goal_value['vector']
                marker_data = {
                    'frame_id':config['fixed_frame'],
                    'pose':{'position':{'x':position[0],'y':position[1],'z':position[2]},
                            'rotation':{'r':0,'p':0,'y':0}
                           },
                    'scale':{'x':0.05,'y':0.05,'z':0.05},
                    'type':'sphere',
                    'color':color
                }
                markers['objective_{0}'.format(idx)] = marker_data
            elif 'quaternion' in goal_value.keys():
                quaternion = goal_value['quaternion']
                # Since the goal doesn't include a position,
                # use the position from the joint_poses structure

            elif 'scalar' in goal_value.keys():
                scalar = goal_value['scalar']
                # Since the goal doesn't include a position,
                # use the position from the joint_poses structure]

            # It isn't something that has an input, do some logic from the objective itself
            elif objective['variant'] == 'position_liveliness':
                arm_index = int(objective['indices'][0])
                joint_index = int(objective['indices'][1])
                position = config['joint_poses'][arm_index][joint_index]['position_global']
                # points = []
                # for i in range(200):
                #     x = np.random.normal(0,objective['shape'][0]/3)
                #     y = np.random.normal(0,objective['shape'][1]/3)
                #     z = np.random.normal(0,objective['shape'][2]/3)
                #     points.append({'x':x,'y':y,'z':z})
                marker_data = {
                    'frame_id':config['fixed_frame'],
                    'pose':{'position':position,
                            'rotation':{'r':0,'p':0,'y':0}
                           },
                    'scale':{'x':objective['shape'][0],'y':objective['shape'][1],'z':objective['shape'][2]},
                    'type':'sphere',
                    'color':color,
                    # 'points':points
                }
                markers['objective_{0}'.format(idx)] = marker_data
            elif objective['variant'] == 'orientation_liveliness':
                arm_index = int(objective['indices'][0])
                joint_index = int(objective['indices'][1])
                position = config['joint_poses'][arm_index][joint_index]['position_global']
                rotation = config['joint_poses'][arm_index][joint_index]['rotation']

                # marker_data = {
                #     'frame_id':config['fixed_frame'],
                #     'pose':{'position':position,
                #             'rotation':{'r':0,'p':0,'y':0}
                #            },
                #     'scale':{'x':objective['shape'][0],'y':objective['shape'][1],'z':objective['shape'][2]},
                #     'type':'sphere',
                #     'color':HIGHLIGHT_COLOR
                # }
                # markers['objective_{0}'.format(objective['tag'])] = marker_data
    return markers

def derive_active_mode(config):
    if config['selected'] and config['selected']['type'] == 'mode':
        return config['selected']['idx']
    else:
        return 0

def derive_active_goals(config):
    if config['selected'] != None and config['selected']['type'] == 'goal':
        return config['selected']['idx']
    else:
        return 0

def derive_target_weights(config):
    return config['modes'][config['active_mode']]['weights']

def derive_target_goals(config):
    return config['goals'][config['active_goals']]['values']
