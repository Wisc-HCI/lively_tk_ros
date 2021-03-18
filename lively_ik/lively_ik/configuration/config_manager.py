from lively_ik.configuration.robot import Robot as PythonRobot
from lively_ik.configuration.collision_graph import CollisionGraph
from lively_ik.configuration.urdf_load import urdf_load_from_string
from lively_ik.configuration.transformations import euler_from_matrix, quaternion_from_euler
from lively_ik.configuration.default import DEFAULT_WEIGHTS
from lively_ik.configuration.updaters.fields import FIELDS
from lively_ik_core import *
import xml.etree.ElementTree as et
from enum import Enum
import numpy.random as nprandom
import numpy as np
from sklearn.neural_network import MLPClassifier, MLPRegressor
from copy import deepcopy

class ConfigManager(object):
    def __init__(self,on_feedback=lambda:None):
        self.on_feedback = on_feedback
        self.history = []
        self.future = []

        self._fields = FIELDS


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
        for non_copy_type in ['config','solver','collision_graph']:
            new_config[non_copy_type] = current[non_copy_type]
        # Force-clear the previous networks and settings
        for field in ['nn_main','nn_jointpoint','nn_main_utd','nn_jointpoint_utd','collision_graph']:
            new_config[field] = self._fields[field]['default']

        changes = set([])
        # Initiate the first process, which sets off processing
        ## Start by creating the collision graph
        collision_graph = self._fields['collision_graph']['derivation'](new_config)
        new_config, new_changes = self.handle_update('collision_graph',collision_graph,new_config,realtime_feedback=True)
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
                if should_derive and self._fields[dependency]['derivation'] != None:
                    new_value = self._fields[dependency]['derivation'](config)
                else:
                    # Revert the derived value to its default
                    new_value = self._fields[dependency]['default']
                config, new_changes = self.handle_update(dependency,new_value,config,realtime_feedback)
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
