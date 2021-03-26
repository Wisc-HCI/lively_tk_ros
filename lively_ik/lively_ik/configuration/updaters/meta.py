import math
import numpy as np
from lively_ik.configuration.transformations import quaternion_from_euler

COLLISION_OBJECT_COLOR = {'r':1.0, 'g':0.55, 'b':0.24, 'a':0.5}; #FF8C3C
PRIMARY_POINT_COLOR = {'r':0.0, 'g':0.88, 'b':0.64, 'a':0.5}; #00E2A3
SECONDARY_POINT_COLOR = {'r':0.13, 'g':0.71, 'b':0.93, 'a':0.5}; #22BDEF
PRIMARY_JOINT_COLOR = {'r':1.0, 'g':0.75, 'b':0.80, 'a':0.5}; #FF13CB
SECONDARY_JOINT_COLOR = {'r':0.58, 'g':0.0, 'b':0.91, 'a':0.5}; #9500E9
DEFAULT_GOAL_COLOR = {'r':1.0, 'g':1.0, 'b':1.0, 'a':0.3}; #ffffff

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
    for marker_set in ['collision_markers','goal_markers','gui_markers']:
        for name,info in config[marker_set].items():
            markers[name] = info
    return markers

def derive_collision_markers(config):
    markers = {}

    # Start with static environment
    for i,collision_cuboid in enumerate(config['static_environment']['cuboids']):
        orientation = quaternion_from_euler(collision_cuboid['rx'], collision_cuboid['ry'], collision_cuboid['rz'], axes='szxy')
        marker_data = {
            'frame_id':'/'+collision_cuboid['coordinate_frame'],
            'pose':{'position':{'x':collision_cuboid['tx'],'y':collision_cuboid['ty'],'z':collision_cuboid['tz']},
                    'orientation':{'w':orientation[0],'x':orientation[1],'y':orientation[2],'z':orientation[3]}
                   },
            'scale':{'x':collision_cuboid['x_halflength']*2,'y':collision_cuboid['y_halflength']*2,'z':collision_cuboid['z_halflength']*2},
            'type':'cube',
            'color':COLLISION_OBJECT_COLOR
        }
        markers['collision_cuboid_{0}'.format(i)] = marker_data

    for i,collision_sphere in enumerate(config['static_environment']['spheres']):
        marker_data = {
            'frame_id':'/'+collision_sphere['coordinate_frame'],
            'pose':{'position':{'x':collision_sphere['tx'],'y':collision_sphere['ty'],'z':collision_sphere['tz']},
                    'orientation':{'w':1,'x':0,'y':0,'z':0}
                   },
            'scale':{'x':collision_sphere['radius']*2,'y':collision_sphere['radius']*2,'z':collision_sphere['radius']*2},
            'type':'sphere',
            'color':COLLISION_OBJECT_COLOR
        }
        markers['collision_sphere_{0}'.format(i)] = marker_data


    if config['show_link_collision']:
        for arm_idx,arm in enumerate(config['joint_names']):
            for joint_idx,joint in enumerate(arm):
                # Joint is the name of the joint, you need to find the name of the link
                if joint_idx+1 < len(arm):
                    joint_1_pose_info = config['joint_poses'][arm_idx][joint_idx]
                    joint_2_pose_info = config['joint_poses'][arm_idx][joint_idx+1]
                    # mdpt = {'x':(joint_1_pose_info['position']['x']+joint_2_pose_info['position']['x'])/2-joint_1_pose_info['position']['x'],
                    #         'y':(joint_1_pose_info['position']['y']+joint_2_pose_info['position']['y'])/2-joint_1_pose_info['position']['y'],
                    #         'z':(joint_1_pose_info['position']['z']+joint_2_pose_info['position']['z'])/2-joint_1_pose_info['position']['z']}
                    length = math.sqrt((joint_1_pose_info['position']['x']-joint_2_pose_info['position']['x'])**2+
                                       (joint_1_pose_info['position']['y']-joint_2_pose_info['position']['y'])**2+
                                       (joint_1_pose_info['position']['z']-joint_2_pose_info['position']['z'])**2
                                      )
                    link_name = config['robot_tree']['joints'][joint]['parent'] # Could be child
                    marker_data = {
                        #Change these values
                        'frame_id':'/'+link_name,
                        # Pose will be determined by how the coordinate frame is structured
                        # Rotation is probably the same as here
                        'pose':{'position':{'x':0,'y':0,'z':-0.5*length},
                                'orientation':{'w':1,'x':0,'y':0,'z':0}
                               },
                        # Figure out scale
                        # from http://wiki.ros.org/rviz/DisplayTypes/Marker
                        # scale.x is diameter in x direction, scale.y in y direction,
                        # by setting these to different values you get an ellipse
                        # instead of a circle. Use scale.z to specify the height.
                        'scale':{'x':config['robot_link_radius']*2,'y':config['robot_link_radius']*2,'z':length},
                        'type':'cylinder',
                        'color':COLLISION_OBJECT_COLOR
                    }
                    markers['collision_{0}'.format(link_name)] = marker_data

    return markers

def derive_goal_markers(config):
    markers = {}
    if config['valid_config']:
        # Add goals, and highlight them if they are selected by that objective
        for idx,goal_value in enumerate(config['target_goals']):
            # Only render non-selected markers, since selected markers should be created in gui_markers
            if not (config['selected'] and config['selected']['type'] == 'objective' and config['selected']['idx'] == idx):
                is_active = False
                if (config['selected'] and config['selected']['type'] == 'goal'):
                    is_active = True
                objective = config['objectives'][idx]

                # First check the goal value itself and discern the type.
                if 'vector' in goal_value:
                    position = goal_value['vector']
                    marker_data = {
                        'frame_id':'/'+config['fixed_frame'],
                        'pose':{'position':{'x':position[0],'y':position[1],'z':position[2]},
                                'orientation':{'w':1,'x':0,'y':0,'z':0}
                               },
                        'scale':{'x':0.05,'y':0.05,'z':0.05},
                        'type':'sphere',
                        'color':PRIMARY_POINT_COLOR if is_active else DEFAULT_GOAL_COLOR
                    }
                    markers['objective_{0}'.format(idx)] = marker_data
                elif 'quaternion' in goal_value:
                    # Get position from joint_poses, since it doesn't have a position
                    position = config['joint_poses'][objective['indices'][0]][objective['indices'][1]]['position']
                    quaternion = goal_value['quaternion']
                    marker_data = {
                        'frame_id':'/'+config['fixed_frame'],
                        'pose':{'position':position,
                                'orientation':{'w':quaternion[0],'x':quaternion[1],'y':quaternion[2],'z':quaternion[3]}
                               },
                        'scale':{'x':0.15,'y':0.07,'z':0.07},
                        'type':'arrow',
                        'color':PRIMARY_POINT_COLOR if is_active else DEFAULT_GOAL_COLOR
                    }
                    markers['objective_{0}'.format(idx)] = marker_data

                elif 'scalar' in goal_value:
                    # Just display the text of the value
                    frame = config['robot_tree']['joints'][config['joint_ordering'][objective['indices'][0]]]['child']
                    marker_data = {
                        'frame_id':'/'+frame,
                        'pose':{'position':{'x':0,'y':0.1,'z':0},
                                'orientation':{'w':1,'x':0,'y':0,'z':0}
                               },
                        'scale':{'x':0.10,'y':0.10,'z':0.10},
                        'type':'text',
                        'data':str(goal_value['scalar']),
                        'color':PRIMARY_JOINT_COLOR if is_active else DEFAULT_GOAL_COLOR
                    }
                    markers['objective_{0}'.format(idx)] = marker_data

                elif 'pose' in goal_value:
                    # Just display the text of the value
                    marker_data = {
                        'frame_id':frame,
                        'pose':{'position':{'x':goal_value['pose'][0][0],'y':goal_value['pose'][0][1],'z':goal_value['pose'][0][2]},
                                'orientation':{'w':goal_value['pose'][1][0],'x':goal_value['pose'][1][1],'y':goal_value['pose'][1][2],'z':goal_value['pose'][1][3]}
                               },
                        'scale':{'x':objective['shape'][0],'y':objective['shape'][1],'z':objective['shape'][2]},
                        'type':'sphere',
                        'color':PRIMARY_POINT_COLOR if is_active else DEFAULT_GOAL_COLOR
                    }
                    markers['objective_{0}'.format(idx)] = marker_data

                    # Since the goal doesn't include a position,
                    # use the position from the joint_poses structure]

                # # It isn't something that has an input, do some logic from the objective itself
                # elif objective['variant'] == 'position_liveliness':
                #     arm_index = int(objective['indices'][0])
                #     joint_index = int(objective['indices'][1])
                #     position = config['joint_poses'][arm_index][joint_index]['position']
                #     marker_data = {
                #         'frame_id':config['fixed_frame'],
                #         'pose':{'position':position,
                #                 'rotation':{'r':0,'p':0,'y':0}
                #                },
                #         'scale':{'x':objective['shape'][0],'y':objective['shape'][1],'z':objective['shape'][2]},
                #         'type':'sphere',
                #         'color':color,
                #         # 'points':points
                #     }
                #     markers['objective_{0}'.format(idx)] = marker_data
                # elif objective['variant'] == 'orientation_liveliness':
                #     arm_index = int(objective['indices'][0])
                #     joint_index = int(objective['indices'][1])
                #     position = config['joint_poses'][arm_index][joint_index]['position']
                #     rotation = config['joint_poses'][arm_index][joint_index]['rotation']

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
