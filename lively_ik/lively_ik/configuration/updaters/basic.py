import xml.etree.ElementTree as et
import numpy as np
from lively_ik.configuration.robot import Robot as PythonRobot
from lively_ik.configuration.urdf_load import urdf_load_from_string
from lively_ik.configuration.transformations import euler_from_matrix, quaternion_from_euler

def search_for_joints_in_chain(root,tree,start=[]):
    children = []
    if start != []:
        # This is if start is the current boundary
        has_added = True
        while has_added:
            has_added = False
            for chain in start:
                last_joint = tree['joints'][chain[-1]]
                last_link = last_joint['child']
                dynamic_children = [name for name,info in tree['joints'].items() if info['parent']==last_link and info['dynamic']]
                if len(dynamic_children) > 0:
                    chain.append(dynamic_children[0])
                    has_added = True
                    children.append(chain)
        return children
    else:
        # Search for the next joint in the chain, and if it has dynamic children, spawn a tree and search
        children_joints = [name for name,info in tree['joints'].items() if info['parent']==root]
        for child_joint in children_joints:
            if tree['joints'][child_joint]['dynamic']:
                children.append([child_joint])
        if len(children) > 0:
            return search_for_joints_in_chain(None,tree,children)
        else:
            static_children = [name for name,info in tree['joints'].items() if info['parent']==root]
            if len(static_children) == 0:
                return children
            else:
                for static_child in static_children:
                    chains = search_for_joints_in_chain(tree['joints'][static_child]['child'],tree,[])
                    for chain in chains:
                        if chain not in children:
                            children.append(chain)
        return children

def search_for_next_fixed_joint(joint,tree):
    child_link = tree['joints'][joint]['child']
    static_children = [name for name,info in tree['joints'].items() if info['parent']==child_link and not info['dynamic']]
    if len(static_children) > 0:
        return static_children[0]
    else:
        dynamic_children = [name for name,info in tree['joints'].items() if info['parent']==child_link and info['dynamic']]
        if len(dynamic_children) > 0:
            for child in dynamic_children:
                search_result = search_for_next_fixed_joint(child,tree)
                if search_result != None:
                    return search_result
    return None

def derive_robot_tree(config):
    if config['parsed_urdf'] == []:
        return {'joints':{},'links':{}}
    tree = {'joints':{},'links':{}}
    for child in config['parsed_urdf']:
        if child.tag == 'link':
            mesh = None
            for link_child in child:
                if link_child.tag == 'visual':
                    for visual_child in link_child:
                        if visual_child.tag == 'geometry':
                            for geometry_child in visual_child:
                                if geometry_child.tag == 'mesh':
                                    try:
                                        mesh = geometry_child.attrib['filename']
                                    except:
                                        print("could not find mesh for {0}".format(child.attrib['name']))
            tree['links'][child.attrib['name']] = {'model':mesh}
        elif child.tag == 'joint':
            parent_link = None
            child_link = None
            joint_limits = [0,0]
            velocity_limit = 0
            multiplier = 1
            mimic = None
            for prop in child:
                if prop.tag == 'parent':
                    parent_link = prop.attrib['link']
                elif prop.tag == 'child':
                    child_link = prop.attrib['link']
                elif prop.tag == 'mimic':
                    mimic = prop.attrib['joint']
                elif prop.tag == 'limit':
                    joint_limits = [float(prop.attrib.get('lower',0)),float(prop.attrib.get('upper',0))]
                    velocity_limit = float(prop.attrib.get('velocity',0))
                    multiplier = float(prop.attrib.get('multiplier',1.0))
            tree['joints'][child.attrib['name']] = {'parent':parent_link,
                                                    'child':child_link,
                                                    'dynamic': child.attrib['type'] != 'fixed',
                                                    'mimic':mimic,
                                                    'multiplier':multiplier,
                                                    'joint_limits':joint_limits,
                                                    'velocity_limit':velocity_limit}
    # Run through the tree and apply any limits of mimic joints
    for joint in tree['joints']:
        if tree['joints'][joint]['mimic'] != None:
            mimic = tree['joints'][joint]['mimic']
            tree['joints'][joint]['velocity_limit'] = tree['joints'][mimic]['velocity_limit']
            multiplier = tree['joints'][mimic]['multiplier']
            if multiplier >= 0:
                tree['joints'][joint]['joint_limits'] = [multiplier*tree['joints'][mimic]['joint_limits'][0],
                                                         multiplier*tree['joints'][mimic]['joint_limits'][1]]
            else:
                tree['joints'][joint]['joint_limits'] = [multiplier*tree['joints'][mimic]['joint_limits'][1],
                                                         multiplier*tree['joints'][mimic]['joint_limits'][0]]
    return tree

def derive_starting_transform(config):
    transform = [0,0,0]
    for i in range(3):
        span = config['base_link_motion_bounds'][i]
        transform[i] = (span[0]+span[1])/2
    return transform

def derive_parsed_urdf(config):
    if config['urdf'] == '':
        result = []
    else:
        try:
            result = et.fromstring(config['urdf'])
        except:
            result = []
    return result

def derive_robot(config):
    if not config['valid_arms']:
        return None
    arms = []
    for i in range(len(config['joint_names'])):
        urdf_robot, arm, arm_c, tree = urdf_load_from_string(config['urdf'], '', '', config['joint_names'][i], config['ee_fixed_joints'][i])
        arms.append(arm)

    return PythonRobot(arms, config['joint_names'], config['joint_ordering'], extra_joints=config['extra_joints'])

def derive_joint_names(config):
    return search_for_joints_in_chain(config['fixed_frame'],config['robot_tree'],[])

def derive_joint_ordering(config):
    ordering = []
    for chain in config['joint_names']:
        for joint in chain:
            if joint not in ordering:
                ordering.append(joint)
    return ordering

def derive_ee_fixed_joints(config):
    joints = [None for chain in config['joint_names']]
    for chain_idx,chain in enumerate(config['joint_names']):
        if len(chain) > 0:
            joints[chain_idx] = search_for_next_fixed_joint(chain[-1],config['robot_tree'])
    return joints

def derive_joint_poses(config):
    if config['robot'] == None:
        return []
    chain_frames = config['robot'].getFrames(config['displayed_state'][0],config['displayed_state'][1])
    joint_poses = []
    for chain_frame in chain_frames:
        chain_poses =  []
        for node_idx in range(len(chain_frame[0])):
            pos = chain_frame[0][node_idx]
            rot_mat = np.zeros((4, 4))
            rot_mat[0:3, 0:3] = chain_frame[1][node_idx]
            rot_mat[3, 3] = 1
            rotation = euler_from_matrix(rot_mat, 'szxy')
            quaternion = quaternion_from_euler(rotation[0],rotation[1],rotation[2],'szxy')
            info = {'position':{'x':pos[0],'y':pos[1],'z':pos[2]},
                    'rotation':{'r':rotation[0],'p':rotation[1],'y':rotation[2]},
                    'quaternion':{'w':quaternion[0],'x':quaternion[1],'y':quaternion[2],'z':quaternion[3]}
                   }
            chain_poses.append(info)
        joint_poses.append(chain_poses)
    return joint_poses

def derive_axis_types(config):
    if config['robot'] == None:
        return []
    num_chains = config['robot'].numChains
    axis_types = []
    for i in range(num_chains):
        arm_axes = []
        chain_len = len(config['robot'].arms[i].axes)
        for j in range(chain_len):
            arm_axes.append(config['robot'].arms[i].axes[j])
        axis_types.append(arm_axes)
    return axis_types

def derive_fixed_frame(config):
    root_candidates = [name for name,info in config['robot_tree']['links'].items()]
    # Find the root (No joints should have that node as a child)
    for candidate in root_candidates:
        if 'base' in candidate:
            return candidate
    for node, info in config['robot_tree']['joints'].items():
        if info['child'] in root_candidates:
            root_candidates.remove(info['child'])
    if len(root_candidates) > 0:
        return root_candidates[0]
    return 'base_link'

def derive_joint_limits(config):
    if config['robot'] == None:
        return []
    return [[pair[0],pair[1]] for pair in config['robot'].bounds]

def derive_joint_types(config):
    if config['robot'] == None:
        return []
    num_chains = config['robot'].numChains
    joint_types = []
    for i in range(num_chains):
        arm_types = []
        chain_len = len(config['robot'].arms[i].joint_types)
        for j in range(chain_len):
            arm_types.append(config['robot'].arms[i].joint_types[j])
        joint_types.append(arm_types)
    return joint_types

def derive_rot_offsets(config):
    if config['robot'] == None:
        return []
    num_chains = config['robot'].numChains
    rot_offsets = []
    for i in range(num_chains):
        arm_offsets = []
        chain_len = len(config['robot'].arms[i].original_rotOffsets)
        for j in range(chain_len):
            d = config['robot'].arms[i].original_rotOffsets[j]
            arm_offsets.append([d[0], d[1], d[2]])
        rot_offsets.append(arm_offsets)
    return rot_offsets

def derive_starting_config(config):
    return ([(limit[0]+limit[1])/2.0 for limit in config['base_link_motion_bounds']],
            [(limit[0]+limit[1])/2.0 for limit in config['joint_limits']])

def derive_velocity_limits(config):
    if config['robot'] == None:
        return []
    return config['robot'].velocity_limits

def derive_disp_offsets(config):
    if config['robot'] == None:
        return []
    num_chains = config['robot'].numChains
    disp_offsets = []
    for i in range(num_chains):
        d = config['robot'].arms[i].dispOffset
        disp_offsets.append([d[0], d[1], d[2]])
    return disp_offsets

def derive_displacements(config):
    if config['robot'] == None:
        return []
    num_chains = config['robot'].numChains
    displacements = []
    for i in range(num_chains):
        arm_displacements = []
        chain_len = len(config['robot'].arms[i].displacements)
        for j in range(chain_len):
            d = config['robot'].arms[i].displacements[j]
            arm_displacements.append([d[0], d[1], d[2]])
        displacements.append(arm_displacements)
    return displacements

def derive_links(config):
    links = []
    for child in config['parsed_urdf']:
        if child.tag == 'link':
            links.append(child.attrib['name'])
    return links

def derive_dynamic_joints(config):
    joints = []
    for child in config['parsed_urdf']:
        if child.tag == 'joint' and child.attrib['type'] != 'fixed':
            joints.append(child.attrib['name'])
    return joints

def derive_fixed_joints(config):
    joints = []
    for child in config['parsed_urdf']:
        if child.tag == 'joint' and child.attrib['type'] == 'fixed':
            joints.append(child.attrib['name'])
    return joints

def derive_extra_joints(config):
    extra_joint_names = []
    for joint in config['joint_ordering']:
        found = False
        for chain in config['joint_names']:
            if joint in chain:
                found = True
        if not found:
            extra_joint_names.append(joint)
    return {name:{'bounds':config['robot_tree']['joints'][name]['joint_limits'],
                  'velocity':config['robot_tree']['joints'][name]['velocity_limit']} for name in extra_joint_names}
