from lively_ik.configuration.robot import Robot as PythonRobot
from lively_ik.configuration.collision_graph import CollisionGraph
from lively_ik.configuration.urdf_load import urdf_load_from_string
from lively_ik.configuration.transformations import euler_from_matrix, quaternion_from_euler
from lively_ik.configuration.default import DEFAULT_CONFIG, DEFAULT_WEIGHTS
from lively_ik.configuration.updaters.fields import FIELDS
from lively_ik_core import *
import xml.etree.ElementTree as et
from enum import Enum
import numpy.random as nprandom
import numpy as np
from sklearn.neural_network import MLPClassifier, MLPRegressor
from copy import deepcopy

class ConfigManager(object):
    def __init__(self,on_feedback=lambda:None,logger=lambda content: print(content)):
        self.on_feedback = on_feedback
        self.log = logger
        self.history = []
        self.future = []

        self._fields = FIELDS


        self._config_keys = ['axis_types','base_link_motion_bounds','ee_fixed_joints',
                             'static_environment','fixed_frame','modes','goals','joint_limits',
                             'joint_names','joint_ordering','joint_types','mode_control',
                             'mode_environment','nn_jointpoint','nn_main','objectives',
                             'states','robot_link_radius','rot_offsets','starting_config',
                             'starting_transform','urdf','velocity_limits','disp_offsets',
                             'displacements']

        self._meta_keys = ['valid_urdf','valid_robot','valid_nn','valid_config','highlights',
                           'valid_solver','links','dynamic_joints','fixed_joints','active_mode','active_goals',
                           'displayed_state','control','show_link_collision','robot_tree','joint_poses',
                           'selected','nn_utd','target_weights','target_goals']

        self._settable_config_keys = ['urdf','fixed_frame','joint_names','ee_fixed_joints',
                                      'joint_ordering','states','starting_config',
                                      'robot_link_radius','static_environment',
                                      'nn_jointpoint','nn_main','objectives',
                                      'modes','goals','base_link_motion_bounds','mode_control',
                                      'mode_environment']

        self._settable_meta_keys = ['control','show_link_collision','selected','highlights','displayed_state',
                                    'active_mode','active_goals','target_weights','target_goals','train_directive']

        self._copy_keys = [key for key in self._fields if key not in ['solver','config','collision_graph']]

        self._update_pairs = set([])
        for field in self._fields:
            # Create a pairs object demarking that
            for dependency in self._fields[field]['dependencies']:
                self._update_pairs = self._update_pairs.union({(dependency,field)})

            # Create a set of all fields that are somehow dependent on that field
            self._fields[field]['full_dependents'] = set([dep for dep in self._fields if field in self._fields[dep]['dependencies']])
            last_added = self._fields[field]['full_dependents'].copy()
            # print(field)
            while len(last_added) > 0:
                # print(f'just added: {last_added}')
                new = set([])
                for added_field in last_added:
                    new = new.union(set([dep for dep in self._fields if added_field in self._fields[dep]['dependencies']]))
                self._fields[field]['full_dependents'] = self._fields[field]['full_dependents'].union(new)
                # print((new,self._fields[field]['full_dependents']))
                last_added = new
            # print(self._fields[field])
            # Create a set of all full dependencies for each field
            # self._fields[field]['full_dependencies'] = self._fields[field]['dependencies'].copy()
            # last_added = self._fields[field]['dependencies'].copy()
            # while len(last_added) > 0:
            #     new = set([])
            #     for added_field in last_added:
            #         new = new.union(self._fields[added_field]['dependencies'])
            #     self._fields[field]['full_dependencies'] = self._fields[field]['full_dependencies'].union(new)
            #     last_added = new.difference(self._fields[field]['full_dependencies'])

        self._front_boundary = set([field for field in self._fields if len(self._fields[field]['dependencies']) == 0])
        # print(f'boundary: {self._front_boundary}')

        # Initial creation
        new_config = {}
        updates = self.get_needed_updates(self._front_boundary.union(set(DEFAULT_CONFIG.keys())))
        self.log(f'Initial Updates: {updates}')
        for update in updates:
            if update in DEFAULT_CONFIG:
                new_config[update] = DEFAULT_CONFIG[update]
            else:
                new_config[update] = self._fields[update]['derivation'](new_config)

        self.__update_config__(new_config)

    @property
    def current(self):
        return self.history[0]

    def __update_config__(self,config):
        self.history = [config]+self.history
        # Limit the history to 20 states back
        if len(self.history) > 20:
            self.history = self.history[0:20]

    def get_needed_updates(self,updated:set):
        update_sequence = []
        immediate_updates = updated.copy()

        graph = self._update_pairs.copy()
        # Trim down graph to only include dependencies of the current set
        # First, create a list of all dependencies
        all_dependents = set([])
        for field in updated:
            all_dependents = all_dependents.union(self._fields[field]['full_dependents'])

        # print(f'all_deps: {all_dependents}')

        # Now remove all edges where that arent dependent
        for pair in self._update_pairs:
            if pair[1] not in all_dependents:
                graph.discard(pair)
            if pair[0] not in updated and pair[0] not in all_dependents:
                graph.discard(pair)

        # print(f'graph: {graph}')

        while len(immediate_updates) > 0:
            update = immediate_updates.pop()
            update_sequence.append(update)
            derivations = [pair[1] for pair in graph if update == pair[0]]
            for derivation in derivations:
                graph.remove((update,derivation))
                if len([pair for pair in graph if derivation == pair[1]]) == 0:
                    immediate_updates.add(derivation)

        if len(graph) > 0:
            self.log(f'\033[93mmissing : {graph}\033[0m')

        return update_sequence

    def update(self,data,meta={},realtime_feedback=False):
        data.update(meta)
        self.future = []
        current = self.current
        new_config = {}
        for key in self._copy_keys:
            new_config[key] = deepcopy(current[key])
        for non_copy_type in ['config','solver','collision_graph']:
            new_config[non_copy_type] = current[non_copy_type]
        changes = set([])
        change_fields = [field for field,value in data.items() if field in self._settable_config_keys + self._settable_meta_keys]

        for change_field in change_fields:
            new_config[change_field] = data[change_field]
            changes.add(change_field)
        needed_updates = self.get_needed_updates(changes)
        self.log(f'needed_updates: {needed_updates}')
        for needed_update in needed_updates:
            if needed_update in data:
                new_config[needed_update] = data[needed_update]
            else:
                new_config[needed_update] = self._fields[needed_update]['derivation'](new_config)
            changes.add(needed_update)
            if realtime_feedback:
                self.__update_config__(new_config)
                self.on_feedback()

        self.__update_config__(new_config)

        return changes

    def train_nn(self):
        self.future = []
        current = self.current
        new_config = {}
        for key in self._copy_keys:
            new_config[key] = deepcopy(current[key])
        for non_copy_type in ['config','solver','collision_graph']:
            new_config[non_copy_type] = None
        for field in ['training_scores','training_frames','training_samples']:
            new_config[field] = []
        new_config['valid_nn'] = False
        for field in ['nn_main','nn_jointpoint']:
            new_config[field] = {'intercepts':[],'coefs':[],'split_point':None}
        self.__update_config__(new_config)
        changes = self.update({'train_directive':True},{},realtime_feedback=True)
        self.history[0]['train_directive']=False
        self.on_feedback()

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
        if config['urdf'] == DEFAULT_CONFIG['urdf']:
            shown['urdf'] = '<default urdf>'
        else:
            shown['urdf'] = '<custom urdf>'
        if config['nn_main'] == DEFAULT_CONFIG['nn_main']:
            shown['nn_main'] = '<default nn_main>'
        else:
            shown['nn_main'] = '<custom nn_main>'
        if config['nn_jointpoint'] == DEFAULT_CONFIG['nn_jointpoint']:
            shown['nn_jointpoint'] = '<default nn_jointpoint>'
        else:
            shown['nn_jointpoint'] = '<custom nn_jointpoint>'
        return shown

    def test_derive(self,field,config=None):
        if config == None:
            config = self.history[0]
        return self._fields[field]['derivation'](config)

    @property
    def data(self):
        return {key:self.history[0][key] for key in self._config_keys}

    @property
    def meta(self):
        meta = {key:self.history[0][key] for key in self._meta_keys}
        if self.history[0]['nn_utd']:
            meta['nn_progress']=100
        else:
            statuses = [
                self.history[0]['valid_nn'],
                self.history[0]['nn_main'] != {'intercepts':[],'coefs':[],'split_point':None},
                self.history[0]['nn_jointpoint'] != {'intercepts':[],'coefs':[],'split_point':None},
                len(self.history[0]['training_scores']) >= 200000,
                len(self.history[0]['training_frames']) >= 200000,
                len(self.history[0]['training_samples']) >= 200000,
                self.history[0]['collision_graph'] != None
            ]
            # print(statuses)
            meta['nn_progress'] = int(sum(statuses) / float(len(statuses)) * 100)
        return meta
