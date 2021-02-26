from lively_ik.configuration.robot import Robot as PythonRobot
from lively_ik.configuration.collision_graph import CollisionGraph
from lively_ik.configuration.urdf_load import urdf_load_from_string
from lively_ik.configuration.transformations import euler_from_matrix, quaternion_from_euler
from lively_ik.configuration.default import DEFAULT_WEIGHTS
from lively_ik_core import *
import xml.etree.ElementTree as et
from enum import Enum
import numpy.random as nprandom
import numpy as np
from sklearn.neural_network import MLPClassifier, MLPRegressor
from copy import deepcopy

def recursive_tolist(obj):
    if isinstance(obj,np.ndarray):
        return obj.tolist()
    elif isinstance(obj,list):
        return [recursive_tolist(o) for o in obj]
    else:
        return obj

class ConfigManager(object):
    def __init__(self,on_feedback=lambda:None):
        self.on_feedback = on_feedback
        self.history = []
        self.future = []

        self._fields = {
            # Rust-Based Fields
            'valid_solver':{
                'default':False,
                'derivation':self.derive_valid_solver,
                'dependencies':['control'],
                'on_change':{},
                'force':True,
                'guards':['valid_config']
            },
            'solver':{
                'default':None,
                'derivation':self.derive_solver,
                'dependencies':['valid_solver'],
                'on_change':{},
                'force':False,
                'guards':['valid_config']
            },
            'valid_config':{
                'default':False,
                'derivation':self.derive_valid_config,
                'dependencies':['solver'],
                'on_change':{},
                'force':True,
                'guards':['valid_nn']
            },
            'config':{
                'default':None,
                'derivation':self.derive_config,
                'dependencies':['valid_config'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot_output','valid_nn']
            },
            # Behavior-Based Fields
            # By default, add simple non-robot specific objectives
            'modes':{
                'default':[{'name':'default','weights':[1.0,1.0,0.1,2.0,5.0]}],
                'derivation':lambda config,rtfb:self.derive_from_default(config,'modes',rtfb),
                'dependencies':['config','target_weights'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'goals':{
                'default':[{'name':'default','values':[{},{},{},{},{}]}],
                'derivation':self.derive_goals,
                'dependencies':['config','target_goals'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'objectives':{
                'default':[{'tag': 'Minimize Velocity', 'variant': 'min_velocity', 'indices': []},
                           {'tag': 'Minimize Acceleration', 'variant': 'min_acceleration', 'indices': []},
                           {'tag': 'Minimize Jerk', 'variant': 'min_jerk', 'indices': []},
                           {'tag': 'Joint Limits', 'variant': 'joint_limits', 'indices': []},
                           {'tag': 'Self-Collision', 'variant': 'nn_collision', 'indices': []}],
                'derivation':lambda config,rtfb:self.derive_from_default(config,'objectives',rtfb),
                'dependencies':['markers','config','target_weights','target_goals'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            # Neural-Network Fields
            'valid_nn':{
                'default':False,
                'derivation':self.derive_valid_nn,
                'dependencies':['nn_progress','config'],
                'on_change':{},
                'force':True,
                'guards':['valid_robot','nn_main_utd','nn_jointpoint_utd']
            },
            'nn_main_utd':{
                'default':False,
                'derivation':lambda config,rtfb:self.derive_from_default(config,'nn_main_utd',rtfb),
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'nn_jointpoint_utd':{
                'default':False,
                'derivation':lambda config,rtfb:self.derive_from_default(config,'nn_jointpoint_utd',rtfb),
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'nn_jointpoint':{
                'default':{'intercepts':[],'coefs':[],'split_point':None},
                'derivation':self.derive_nn_jointpoint,
                'dependencies':['nn_progress','valid_nn'],
                'on_change':{'nn_jointpoint_utd':True},
                'force':True,
                'guards':['valid_robot']
            },
            'nn_main':{
                'default':{'intercepts':[],'coefs':[],'split_point':None},
                'derivation':self.derive_nn_main,
                'dependencies':['nn_progress','valid_nn'],
                'on_change':{'nn_main_utd':True},
                'force':True,
                'guards':['valid_robot']
            },
            'training_scores':{
                'default':[],
                'derivation':self.derive_training_scores,
                'dependencies':['nn_progress'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'training_frames':{
                'default':[],
                'derivation':self.derive_training_frames,
                'dependencies':['nn_progress','nn_jointpoint'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'training_samples':{
                'default':[],
                'derivation':self.derive_training_samples,
                'dependencies':['nn_progress','training_scores','nn_main','training_frames'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'collision_graph':{
                'default':None,
                'derivation':self.derive_collision_graph,
                'dependencies':['nn_progress','training_samples'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'states':{
                'default':[],
                'derivation':lambda config,rtfb:self.derive_from_default(config,'states',rtfb),
                'dependencies':[],
                'on_change':{'valid_nn':False,
                             'nn_main':{'intercepts':[],'coefs':[],'split_point':None},
                             'nn_jointpoint':{'intercepts':[],'coefs':[],'split_point':None},
                             'nn_main_utd':False,
                             'nn_jointpoint_utd':False,
                             'collision_graph':None,
                             'training_samples':[],
                             'training_frames':[],
                             'training_scores':[]
                             },
                'force':False,
                'guards':['valid_robot']
            },
            'static_environment':{
                'default':{
                        'cuboids':[],
                        'spheres':[],
                        'pcs':[]
                    },
                'derivation':lambda config,rtfb:self.derive_from_default(config,'static_environment',rtfb),
                'dependencies':['markers'],
                'on_change':{'valid_nn':False,
                             'nn_main':{'intercepts':[],'coefs':[],'split_point':None},
                             'nn_jointpoint':{'intercepts':[],'coefs':[],'split_point':None},
                             'nn_main_utd':False,
                             'nn_jointpoint_utd':False,
                             'collision_graph':None,
                             'training_samples':[],
                             'training_frames':[],
                             'training_scores':[]
                             },
                'force':False,
                'guards':['valid_urdf']
            },
            'robot_link_radius':{
                'default':0.05,
                'derivation':lambda config,rtfb:self.derive_from_default(config,'robot_link_radius',rtfb),
                'dependencies':['markers'],
                'on_change':{'valid_nn':False,
                             'nn_main':{'intercepts':[],'coefs':[],'split_point':None},
                             'nn_jointpoint':{'intercepts':[],'coefs':[],'split_point':None},
                             'nn_main_utd':False,
                             'nn_jointpoint_utd':False,
                             'collision_graph':None,
                             'training_samples':[],
                             'training_frames':[],
                             'training_scores':[]
                             },
                'force':False,
                'guards':[]
            },
            # Robot-Derived Fields
            'starting_config':{
                'default':[],
                'derivation':self.derive_starting_config,
                'dependencies':['displayed_state','config','goals'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'valid_robot_output':{
                'default':[],
                'derivation':self.derive_valid_robot_output,
                'dependencies':['starting_config','config'],
                'on_change':{},
                'force':True,
                'guards':['valid_robot']
            },
            'axis_types':{
                'default':[],
                'derivation':self.derive_axis_types,
                'dependencies':['valid_robot_output'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'joint_limits':{
                'default':[],
                'derivation':self.derive_joint_limits,
                'dependencies':['valid_robot_output'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'joint_types':{
                'default':[],
                'derivation':self.derive_joint_types,
                'dependencies':['valid_robot_output'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'velocity_limits':{
                'default':[],
                'derivation':self.derive_velocity_limits,
                'dependencies':['valid_robot_output'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'rot_offsets':{
                'default':[],
                'derivation':self.derive_rot_offsets,
                'dependencies':['valid_robot_output'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'disp_offsets':{
                'default':[],
                'derivation':self.derive_disp_offsets,
                'dependencies':['valid_robot_output'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'displacements':{
                'default':[],
                'derivation':self.derive_displacements,
                'dependencies':['valid_robot_output'],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            # Robot-Based Fields
            'valid_robot':{
                'default':False,
                'derivation':self.derive_valid_robot,
                'dependencies':['axis_types','joint_types','joint_limits',
                                'rot_offsets','velocity_limits','disp_offsets',
                                'displacements','states','objectives','modes',
                                'goals','joint_poses'],
                'on_change':{},
                'force':True,
                'guards':['valid_arms']
            },
            'robot':{
                'default':None,
                'derivation':self.derive_robot,
                'dependencies':['valid_robot'],
                'on_change':{'nn_main_utd':False,'nn_jointpoint_utd':False},
                'force':False,
                'guards':['valid_arms']
            },
            'valid_arms':{
                'default':False,
                'derivation':self.derive_valid_arms,
                'dependencies':['robot'],
                'on_change':{},
                'force':True,
                'guards':['valid_urdf']
            },
            'extra_joints':{
                'default':{},
                'derivation':self.derive_extra_joints,
                'dependencies':['valid_arms'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'ee_fixed_joints':{
                'default':[],
                'derivation':self.derive_ee_fixed_joints,
                'dependencies':['valid_arms'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'fixed_frame':{
                'default':'base_link',
                'derivation':self.derive_fixed_frame,
                'dependencies':['joint_names'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'joint_ordering':{
                'default':[],
                'derivation':self.derive_joint_ordering,
                'dependencies':['extra_joints'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'joint_names':{
                'default':[],
                'derivation':self.derive_joint_names,
                'dependencies':['joint_ordering','extra_joints'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            # Simple URDF Fields
            'links':{
                'default':[],
                'derivation':self.derive_links,
                'dependencies':['show_link_collision','markers'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'fixed_joints':{
                'default':[],
                'derivation':self.derive_fixed_joints,
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'dynamic_joints':{
                'default':[],
                'derivation':self.derive_dynamic_joints,
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'robot_tree':{
                'default':[],
                'derivation':self.derive_robot_tree,
                'dependencies':['fixed_frame','joint_ordering','ee_fixed_joints'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'valid_urdf':{
                'default':False,
                'derivation':self.derive_valid_urdf,
                'dependencies':['links','fixed_joints','dynamic_joints','robot_tree'],
                'on_change':{},
                'force':True,
                'guards':[]
            },
            'parsed_urdf':{
                'default':None,
                'derivation':self.derive_parsed_urdf,
                'dependencies':['valid_urdf'],
                'on_change':{},
                'force':False,
                'guards':[]
            },
            'urdf':{
                'default':'<?xml version="1.0" ?><robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"/><joint name="default_joint" type="fixed"><parent link="base_link" /><child link="default_link" /><origin xyz="0 0 0" rpy="0 0 0" /></joint><link name="default_link"/></robot>',
                'derivation':lambda config,rtfb:self.derive_from_default(config,'urdf',rtfb),
                'dependencies':['parsed_urdf'],
                'on_change':{},
                'force':False,
                'guards':[]
            },
            # Misc Fields
            'base_link_motion_bounds':{
                'default':[[0,0],[0,0],[0,0]],
                'derivation':lambda config,rtfb:self.derive_from_default(config,'base_link_motion_bounds',rtfb),
                'dependencies':['config'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'mode_control':{
                'default':'absolute',
                'derivation':lambda config,rtfb:self.derive_from_default(config,'mode_control',rtfb),
                'dependencies':['config'],
                'on_change':{},
                'force':False,
                'guards':[]
            },
            'mode_environment':{
                'default':'ECAA',
                'derivation':lambda config,rtfb:self.derive_from_default(config,'mode_environment',rtfb),
                'dependencies':['config'],
                'on_change':{},
                'force':False,
                'guards':[]
            },
            # GUI Fields
            'displayed_state':{
                'default':[],
                'derivation':self.derive_displayed_state,
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':[]
            },
            'control':{
                'default':'manual',
                'derivation':self.derive_control,
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':[]
            },
            'selected':{
                'default':None,
                'derivation':lambda config,rtfb:self.derive_from_default(config,'selected',rtfb),
                'dependencies':['markers','active_mode'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'markers':{
                'default':{},
                'derivation':self.derive_markers,
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'show_link_collision':{
                'default':False,
                'derivation':lambda config,rtfb:self.derive_from_default(config,'show_link_collision',rtfb),
                'dependencies':['markers'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'active_mode':{
                'default':'default',
                'derivation':self.derive_active_mode,
                'dependencies':['target_weights'],
                'on_change':{},
                'force':False,
                'guards':['valid_config']
            },
            'active_goals':{
                'default':'default',
                'derivation':self.derive_active_goals,
                'dependencies':['target_goals'],
                'on_change':{},
                'force':False,
                'guards':['valid_config']
            },
            'joint_poses':{
                'default':[],
                'derivation':self.derive_joint_poses,
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'target_weights':{
                'default':[1.0,1.0,0.1,2.0,5.0],
                'derivation':self.derive_target_weights,
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'target_goals':{
                'default':[{},{},{},{},{}],
                'derivation':self.derive_target_goals,
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':['valid_robot']
            },
            'highlights':{
                'default':[],
                'derivation':lambda config,rtfb:self.derive_from_default(config,'highlights',rtfb),
                'dependencies':['markers'],
                'on_change':{},
                'force':False,
                'guards':['valid_urdf']
            },
            'nn_progress':{
                'default':0,
                'derivation':self.derive_nn_progress,
                'dependencies':[],
                'on_change':{},
                'force':False,
                'guards':['valid_robot_output']
            }
        }


        self._config_keys = ['axis_types','base_link_motion_bounds','ee_fixed_joints',
                             'static_environment','fixed_frame','modes','goals','joint_limits',
                             'joint_names','joint_ordering','joint_types','mode_control',
                             'mode_environment','nn_jointpoint','nn_main','objectives',
                             'states','robot_link_radius','rot_offsets','starting_config',
                             'urdf','velocity_limits','disp_offsets','displacements']

        self._meta_keys = ['valid_urdf','valid_robot','valid_nn','valid_config','highlights',
                           'valid_solver','links','dynamic_joints','fixed_joints','active_mode','active_goals',
                           'displayed_state','control','show_link_collision','robot_tree','nn_progress','joint_poses',
                           'selected','valid_robot_output','nn_main_utd','nn_jointpoint_utd','target_weights','target_goals']

        self._settable_config_keys = ['urdf','fixed_frame','joint_names','ee_fixed_joints',
                                      'joint_ordering','states','starting_config',
                                      'robot_link_radius','static_environment',
                                      'nn_jointpoint','nn_main','objectives',
                                      'modes','goals','base_link_motion_bounds','mode_control',
                                      'mode_environment']

        self._settable_meta_keys = ['control','show_link_collision','selected','highlights','displayed_state',
                                    'active_mode','active_goals','target_weights','target_goals']

        self._copy_keys = [key for key in self._fields if key not in ['solver','config','collision_graph']]

        initial = {field:value['default'] for field,value in self._fields.items()}
        self.history.append(initial)

    def search_dependencies(self,keys):
        deps = set(keys)
        for key in keys:
            deps = deps.union(self.search_dependencies(self._fields[key]['dependencies']))
        return deps

    @property
    def current(self):
        return self.history[0]

    def load(self,data,meta={}):
        data.update(meta)
        self.future = []
        current = self.current
        new_config = {}
        for key in self._copy_keys:
            new_config[key] = deepcopy(current[key])
        for non_copy_type in ['config','solver','collision_graph']:
            new_config[non_copy_type] = current[non_copy_type]

        changes = set([])

        for property in self._settable_config_keys + self._settable_meta_keys:
            if property in data:
                # print('Update {0} to {1}'.format(property,data[property]))
                new_config, new_changes = self.handle_update(property,data[property],new_config,realtime_feedback=False)
                changes = changes.union(new_changes)

        self.__update_config__(new_config)
        return changes

    def train_nn(self):
        current = self.current
        new_config = {}
        for key in self._copy_keys:
            new_config[key] = deepcopy(current[key])
        for rust_type in ['config','solver']:
            new_config[rust_type] = current[rust_type]
        # Force-clear the previous networks and settings
        for field in ['nn_main','nn_jointpoint','nn_main_utd','nn_jointpoint_utd','collision_graph']:
            new_config = self.derive_from_default(new_config,field,True)
        # Initiate the first process, which sets off processing
        new_config = self.derive_collision_graph(new_config,realtime_feedback=True)
        self.__update_config__(new_config)

    def __update_config__(self,config):
        self.history = [config]+self.history
        # Limit the history to 20 states back
        if len(self.history) > 20:
            self.history = self.history[0:20]

    def undo(self):
        if len(self.history) > 0:
            future = self.history.pop(0)
            self.future.append(future)

    def redo(self):
        if len(self.future) > 0:
            current = self.future.pop(-1)
            self.history = [current] + self.history

    def derive_from_default(self,config,field,realtime_feedback=False):
        changes = {field}
        config[field] = self._fields[field]['default']
        for dependency in self._fields[field]['dependencies']:
            config, new_changes = self.derive_from_default(config,dependency,realtime_feedback)
            changes = changes.union(new_changes)
        return config, changes

    def simplified(self,config=None):
        if config == None:
            config = self.data
        shown = {key:value for key,value in config.items() if key != 'urdf'}
        if config['urdf'] == self._fields['urdf']['default']:
            shown['urdf'] = '<default urdf>'
        else:
            shown['urdf'] = '<custom urdf>'
        return shown

    # Macro for handling updates
    def handle_update(self,property,value,config,realtime_feedback=False):
        changes = {property}
        if config[property] != value or self._fields[property]['force']:
            # Set the value
            config[property] = value
            if realtime_feedback:
                self.__update_config__(config)
            # Run any updates from on_change of property
            for field,change in self._fields[property]['on_change'].items():
                # There are no checks here, just make the change
                # print('\033[93mForcing change of {0} to {1}\033[0m'.format(field,change))
                config, new_changes = self.handle_update(field,change,config,realtime_feedback)
                changes = changes.union(new_changes)
                if realtime_feedback:
                    self.__update_config__(config)
            for dependency in self._fields[property]['dependencies']:
                # Prevent the derivation of subsequent
                # values if any of their guards fail
                should_derive = True
                for guard in self._fields[dependency]['guards']:
                    if should_derive and not config[guard] == True:
                        print('\033[93mPrevented setting {0} by guard {1}\033[0m'.format(dependency,guard))
                        should_derive = False
                if should_derive:
                    config, new_changes = self._fields[dependency]['derivation'](config,realtime_feedback)
                    changes = changes.union(new_changes)
                    if realtime_feedback:
                        self.__update_config__(config)
                else:
                    # Revert the derived value to its default
                    config, new_changes = self.derive_from_default(config,dependency,True)
                    changes = changes.union(new_changes)
                    if realtime_feedback:
                        self.__update_config__(config)
        return config, changes

    @property
    def data(self):
        return {key:self.history[0][key] for key in self._config_keys}

    @property
    def meta(self):
        return {key:self.history[0][key] for key in self._meta_keys}

    # Property derivers

    def derive_config(self,config,realtime_feedback=False):
        return self.handle_update('config',parse_config_data({key:config[key] for key in self._config_keys}),config,realtime_feedback)

    def derive_valid_config(self,config,realtime_feedback=False):
        return self.handle_update('valid_config',config['config'] != None,config,realtime_feedback)

    def derive_solver(self,config,realtime_feedback=False):
        return self.handle_update('solver',LivelyIK(config['config']),config,realtime_feedback)

    def derive_valid_solver(self,config,realtime_feedback=False):
        return self.handle_update('valid_solver',config['solver'] != None,config,realtime_feedback)

    def derive_valid_nn(self,config,realtime_feedback=False):
        return self.handle_update('valid_nn',config['nn_main'] != self._fields['nn_main']['default'] and config['nn_jointpoint'] != self._fields['nn_jointpoint']['default'],config,realtime_feedback)

    def derive_valid_robot_output(self,config,realtime_feedback=False):
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
        return self.handle_update('valid_robot_output',passes,config,realtime_feedback)

    def derive_robot(self,config,realtime_feedback=False):
        arms = []
        for i in range(len(config['joint_names'])):
            urdf_robot, arm, arm_c, tree = urdf_load_from_string(config['urdf'], '', '', config['joint_names'][i], config['ee_fixed_joints'][i])
            arms.append(arm)

        robot = PythonRobot(arms, config['joint_names'], config['joint_ordering'], extra_joints=config['extra_joints'])
        return self.handle_update('robot',robot,config,realtime_feedback)

    def derive_valid_robot(self,config,realtime_feedback=False):
        return self.handle_update('valid_robot',config['robot'] != None,config,realtime_feedback)

    def derive_parsed_urdf(self,config,realtime_feedback=False):
        try:
            result = et.fromstring(config['urdf'])
        except:
            result = None
        return self.handle_update('parsed_urdf',result,config,realtime_feedback)

    def derive_valid_urdf(self,config,realtime_feedback=False):
        return self.handle_update('valid_urdf',config['parsed_urdf'] != None,config,realtime_feedback)

    def derive_valid_arms(self,config,realtime_feedback=False):
        if len(config['joint_names']) == len(config['ee_fixed_joints']) and len(config['joint_names']) > 0 and len(config['joint_ordering']) > 0:
            for chain in config['joint_names']:
                for joint in chain:
                    if joint not in config['joint_ordering']:
                        return self.handle_update('valid_arms',False,config,realtime_feedback)
            result = True
        else:
            result = False
        return self.handle_update('valid_arms',result,config,realtime_feedback)

    def derive_robot_tree(self,config,realtime_feedback=False):
        tree = {}
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
                tree[child.attrib['name']] = {'type':'link','model':mesh}
            elif child.tag == 'joint':
                parent_link = None
                child_link = None
                for prop in child:
                    if prop.tag == 'parent':
                        parent_link = prop.attrib['link']
                    elif prop.tag == 'child':
                        child_link = prop.attrib['link']
                tree[child.attrib['name']] = {'type':'joint','parent':parent_link,'child':child_link,'dynamic': child.attrib['type'] != 'fixed'}
        return self.handle_update('robot_tree',tree,config,realtime_feedback)

    def derive_goals(self,config,realtime_feedback=False):
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
        return self.handle_update('goals',goals,config,realtime_feedback)

    def derive_joint_names(self,config,realtime_feedback=False):
        names = self.search_for_chain(config['fixed_frame'],config['robot_tree'])
        return self.handle_update('joint_names',names,config,realtime_feedback)

    def derive_joint_ordering(self,config,realtime_feedback=False):
        ordering = []
        for chain in config['joint_names']:
            for joint in chain:
                if joint not in ordering:
                    ordering.append(joint)
        return self.handle_update('joint_ordering',ordering,config,realtime_feedback)

    def derive_ee_fixed_joints(self,config,realtime_feedback=False):
        joints = [None for chain in config['joint_names']]
        final_dynamic_joints = [chain[-1] if len(chain) > 0 else [] for chain in config['joint_names']]
        for name,info in config['robot_tree'].items():
            if info['type'] == 'joint' and name in final_dynamic_joints:
                # Get the child link
                child_link = info['child']
                # Find the next fixed joint in the chain
                link_children = [name for name,info in config['robot_tree'].items() if info['type'] == 'joint' and info['parent'] == child_link and not info['dynamic']]
                if len(link_children) > 0:
                    # Just select the first one for the default
                    idx = final_dynamic_joints.index(name)
                    joints[idx] = link_children[0]
        return self.handle_update('ee_fixed_joints',joints,config,realtime_feedback)

    def derive_joint_poses(self,config,realtime_feedback=False):
        chain_frames = config['robot'].getFrames(config['displayed_state'])
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
        return self.handle_update('joint_poses',joint_poses,config,realtime_feedback)

    def derive_axis_types(self,config,realtime_feedback=False):
        num_chains = config['robot'].numChains
        axis_types = []
        for i in range(num_chains):
            arm_axes = []
            chain_len = len(config['robot'].arms[i].axes)
            for j in range(chain_len):
                arm_axes.append(config['robot'].arms[i].axes[j])
            axis_types.append(arm_axes)
        return self.handle_update('axis_types',axis_types,config,realtime_feedback)

    def derive_fixed_frame(self,config,realtime_feedback=False):
        root_candidates = [name for name,info in config['robot_tree'].items() if info['type'] == 'link']
        # Find the root (No joints should have that node as a child)
        for node, info in config['robot_tree'].items():
            if info['type'] == 'joint' and info['child'] in root_candidates:
                root_candidates.remove(info['child'])
        return self.handle_update('fixed_frame',root_candidates[0],config,realtime_feedback)

    def derive_joint_limits(self,config,realtime_feedback=False):
        return self.handle_update('joint_limits',[[pair[0],pair[1]] for pair in config['robot'].bounds],config,realtime_feedback)

    def derive_joint_types(self,config,realtime_feedback=False):
        num_chains = config['robot'].numChains
        joint_types = []
        for i in range(num_chains):
            arm_types = []
            chain_len = len(config['robot'].arms[i].joint_types)
            for j in range(chain_len):
                arm_types.append(config['robot'].arms[i].joint_types[j])
            joint_types.append(arm_types)
        return self.handle_update('joint_types',joint_types,config,realtime_feedback)

    def derive_nn_main(self,config,realtime_feedback=False):
        layer_width = 15
        num_layers = 5

        # Train Main NN
        hls = []
        for i in range(num_layers):
            hls.append(layer_width)
        hls = tuple(hls)
        clf_main = MLPRegressor(solver='adam', alpha=1,
                                hidden_layer_sizes=hls, max_iter=300000, verbose=True,
                                learning_rate='adaptive')

        clf_main.fit(config['training_samples'], config['training_scores'])
        split_point = self.find_optimal_split_point(clf_main,config['robot'],config['collision_graph'],2000,jointpoint=False)

        intercepts = recursive_tolist(clf_main.intercepts_)
        coefs = recursive_tolist(clf_main.coefs_)

        return self.handle_update('nn_main',{'intercepts':intercepts,'coefs':coefs,'split_point':split_point},config,realtime_feedback)

    def derive_nn_jointpoint(self,config,realtime_feedback=False):
        layer_width = 15
        num_layers = 5

        # Train Jointpoint NN
        hls = []
        for i in range(num_layers):
            hls.append(layer_width)
        hls = tuple(hls)
        clf_jp = MLPRegressor(solver='adam', alpha=1,
                                hidden_layer_sizes=hls, max_iter=300000, verbose=True,
                                learning_rate='adaptive')

        clf_jp.fit(config['training_frames'], config['training_scores'])
        split_point = self.find_optimal_split_point(clf_jp,config['robot'],config['collision_graph'],2000,jointpoint=True)

        intercepts = recursive_tolist(clf_jp.intercepts_)
        coefs = recursive_tolist(clf_jp.coefs_)

        return self.handle_update('nn_jointpoint',{'intercepts':intercepts,'coefs':coefs,'split_point':split_point},config,realtime_feedback)

    def derive_training_scores(self,config,realtime_feedback=False):
        scores = []
        for sample in config['training_samples']:
            frames = config['robot'].getFrames(sample)
            scores.append(config['collision_graph'].get_collision_score(frames))
        return self.handle_update('training_scores',scores,config,realtime_feedback)

    def derive_collision_graph(self,config,realtime_feedback=False):
        return self.handle_update('collision_graph',CollisionGraph(config, config['robot'], sample_states=config['states']),config,realtime_feedback)

    def derive_training_frames(self,config,realtime_feedback=False):
        training_frames = []
        for sample in config['training_samples']:
            frames = config['robot'].getFrames(sample)
            training_frames.append(self.frames_to_jt_pt_vec(frames))
        return self.handle_update('training_frames',training_frames,config,realtime_feedback)

    def derive_training_samples(self,config,realtime_feedback=False):
        samples = []
        for i in range(200000):
            try:
                state = config['states'][i]
            except:
                state = []
                for b in config['robot'].bounds:
                    rand = nprandom.uniform(low=b[0], high=b[1], size=(1))[0]
                    state.append(rand)
            samples.append(state)
        return self.handle_update('training_samples',samples,config,realtime_feedback)

    def derive_rot_offsets(self,config,realtime_feedback=False):
        num_chains = config['robot'].numChains
        rot_offsets = []
        for i in range(num_chains):
            arm_offsets = []
            chain_len = len(config['robot'].arms[i].original_rotOffsets)
            for j in range(chain_len):
                d = config['robot'].arms[i].original_rotOffsets[j]
                arm_offsets.append([d[0], d[1], d[2]])
            rot_offsets.append(arm_offsets)
        return self.handle_update('rot_offsets',rot_offsets,config,realtime_feedback)

    def derive_starting_config(self,config,realtime_feedback=False):
        return self.handle_update('starting_config',[(limit[0]+limit[1])/2.0 for limit in config['joint_limits']],config,realtime_feedback)

    def derive_velocity_limits(self,config,realtime_feedback=False):
        return self.handle_update('velocity_limits',config['robot'].velocity_limits,config,realtime_feedback)

    def derive_disp_offsets(self,config,realtime_feedback=False):
        num_chains = config['robot'].numChains
        disp_offsets = []
        for i in range(num_chains):
            d = config['robot'].arms[i].dispOffset
            disp_offsets.append([d[0], d[1], d[2]])
        return self.handle_update('disp_offsets',disp_offsets,config,realtime_feedback)

    def derive_displacements(self,config,realtime_feedback=False):
        num_chains = config['robot'].numChains
        displacements = []
        for i in range(num_chains):
            arm_displacements = []
            chain_len = len(config['robot'].arms[i].displacements)
            for j in range(chain_len):
                d = config['robot'].arms[i].displacements[j]
                arm_displacements.append([d[0], d[1], d[2]])
            displacements.append(arm_displacements)
        return self.handle_update('displacements',displacements,config,realtime_feedback)

    def derive_links(self,config,realtime_feedback=False):
        links = []
        for child in config['parsed_urdf']:
            if child.tag == 'link':
                links.append(child.attrib['name'])
        return self.handle_update('links',links,config,realtime_feedback)

    def derive_dynamic_joints(self,config,realtime_feedback=False):
        joints = []
        for child in config['parsed_urdf']:
            if child.tag == 'joint' and child.attrib['type'] != 'fixed':
                joints.append(child.attrib['name'])
        return self.handle_update('dynamic_joints',joints,config,realtime_feedback)

    def derive_fixed_joints(self,config,realtime_feedback=False):
        joints = []
        for child in config['parsed_urdf']:
            if child.tag == 'joint' and child.attrib['type'] == 'fixed':
                joints.append(child.attrib['name'])
        return self.handle_update('fixed_joints',joints,config,realtime_feedback)

    def derive_extra_joints(self,config,realtime_feedback=False):
        extra_joint_names = []
        for joint in config['joint_ordering']:
            found = False
            for chain in config['joint_names']:
                if joint in chain:
                    found = True
            if not found:
                extra_joint_names.append(joint)
        extra_joints = {name:{'bounds':[0,0],'velocity':0} for name in extra_joint_names}
        for child in config['parsed_urdf']:
            if child.tag == 'joint' and child.attrib['type'] != 'fixed' and child.attrib['name'] in extra_joint_names:
                for attrib in child:
                    if attrib.tag == 'limit':
                        extra_joints[child.attrib['name']]['bounds'][0] = float(attrib.attrib['lower'])
                        extra_joints[child.attrib['name']]['bounds'][1] = float(attrib.attrib['upper'])
                        extra_joints[child.attrib['name']]['velocity'] = float(attrib.attrib['velocity'])
        return self.handle_update('extra_joints',extra_joints,config,realtime_feedback)

    def derive_displayed_state(self,config,realtime_feedback=False):
        if config['selected'] != None:
            if config['selected']['type'] == 'starting_config':
                return self.handle_update('displayed_state',config['starting_config'],config,realtime_feedback)
            elif config['selected']['type'] == 'collision_state' and config['selected']['idx'] != None:
                return self.handle_update('displayed_state',config['states'][idx],config,realtime_feedback)
        else:
            return self.handle_update('displayed_state',config['starting_config'],config,realtime_feedback)

    def derive_control(self,config,realtime_feedback=False):
        if config['valid_solver']:
            control = 'solve'
        else:
            control = self._fields['control']['default']
        return self.handle_update('control',control,config,realtime_feedback)

    def derive_markers(self,config,realtime_feedback=False):
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


        return self.handle_update('markers',markers,config,realtime_feedback)

    def derive_active_mode(self,config,realtime_feedback=False):
        if config['selected'] and config['selected']['type'] == 'mode':
            mode = config['modes'][config['selected']['idx']]['name']
        else:
            mode = 'default'
        return self.handle_update('active_mode',mode,config,realtime_feedback)

    def derive_active_goals(self,config,realtime_feedback=False):
        if config['selected'] and config['selected']['type'] == 'goal':
            goals = config['goals'][config['selected']['idx']]['name']
        else:
            goals = 'default'
        return self.handle_update('active_goals',goals,config,realtime_feedback)

    def derive_target_weights(self,config,realtime_feedback=False):
        mode = [mode for mode in config['modes'] if mode['name'] == config['active_mode']][0]
        return self.handle_update('target_weights',mode['weights'],config,realtime_feedback)

    def derive_target_goals(self,config,realtime_feedback=False):
        goal = [goal for goal in config['goals'] if goal['name'] == config['active_goals']][0]
        return self.handle_update('target_goals',goal['values'],config,realtime_feedback)

    def derive_nn_progress(self,config,realtime_feedback=False):
        statuses = [
            config['valid_nn'],
            config['nn_main'] != self._fields['nn_main']['default'],
            config['nn_jointpoint'] != self._fields['nn_jointpoint']['default'],
            len(config['training_scores']) >= 200000,
            len(config['training_frames']) >= 200000,
            len(config['training_samples']) >= 200000,
            config['collision_graph'] != None
        ]
        # print(statuses)
        percent = int(sum(statuses) / float(len(statuses)) * 100)
        return self.handle_update('nn_progress',percent,config,realtime_feedback)

    @staticmethod
    def search_for_chain(root,tree):
        children = []
        joints_with_root_parent = [name for name,info in tree.items() if info['type']=='joint' and info['parent']==root]
        for joint in joints_with_root_parent:
            if tree[joint]['dynamic']:
                chains = ConfigManager.search_for_chain(tree[joint]['child'],tree)
                for chain in chains:
                    children.append([joint]+chain)
            else:
                children.append([])
        return children

    @staticmethod
    def frames_to_jt_pt_vec(all_frames):
        out_vec = []
        for frames in all_frames:
            jt_pts = frames[0]
            for j in jt_pts:
                out_vec.append(j[0])
                out_vec.append(j[1])
                out_vec.append(j[2])
        return out_vec

    @staticmethod
    def find_optimal_split_point(clf,robot,graph,num_samples=2000,jointpoint=False):
        predictions = []
        ground_truths = []
        for i in range(num_samples):
            if jointpoint:
                state = []
                for b in robot.bounds:
                    rand = nprandom.uniform(low=b[0], high=b[1], size=(1))[0]
                    state.append(rand)
                frames = robot.getFrames(state)
                jt_pt_vec = ConfigManager.frames_to_jt_pt_vec(frames)
                pred = clf.predict([jt_pt_vec])
            else:
                state = []
                for b in robot.bounds:
                    rand = nprandom.uniform(low=b[0], high=b[1], size=(1))[0]
                    state.append(rand)
                pred = clf.predict([state])
            ground_truth = graph.get_collision_score_of_state(state)
            predictions.append(pred)
            ground_truths.append(ground_truth)
        best_split = 5.0
        best_split_score = 100000000000.
        split_point = 8.0
        best_false_positive_count = 0.0
        best_false_negative_count = 0.0
        while split_point > 0.0:
            false_positive_count = 0.
            false_negative_count = 0.
            total_count = 0.
            for i in range(num_samples):
                if predictions[i] >= split_point and ground_truths[i] < 5.0:
                    false_negative_count += 1.0
                elif predictions[i] < split_point and ground_truths[i] >= 5.0:
                    false_positive_count += 1.0
                total_count += 1.0
            split_score = 2.0*(false_positive_count / total_count) + (false_negative_count / total_count)
            if split_score < best_split_score:
                best_split_score = split_score
                best_split = split_point
                best_false_negative_count = false_negative_count
                best_false_positive_count = false_positive_count
            split_point -= 0.01
        return best_split
