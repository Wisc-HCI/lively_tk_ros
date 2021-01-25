from lively_ik.configuration.robot import Robot as PythonRobot
from lively_ik.configuration.collision_graph import CollisionGraph
from lively_ik.configuration.urdf_load import urdf_load_from_string
from lively_ik_core import *
import xml.etree.ElementTree as et
from enum import Enum
import numpy.random as nprandom
from sklearn.neural_network import MLPClassifier, MLPRegressor

class ConfigManager(object):
    def __init__(self):

        self._dependencies = {
            'urdf':{'base_link_motion_bounds','mode_control','mode_environment',
                    'robot_link_radius','static_environment'},
            'robot':{'objectives','goals'},
            'parsed_urdf':{'ee_fixed_joints','joint_names','joint_ordering','states'}
        }
        self._defaults = {
            'solver':None,
            'config':None,
            'robot':None,
            'parsed_urdf':None,
            'axis_types':[],
            'base_link_motion_bounds':[[0,0],[0,0],[0,0]],
            'ee_fixed_joints':[],
            'static_environment':{
                    'cuboids':[],
                    'spheres':[],
                    'pcs':[]
                },
            'fixed_frame':'base_link',
            'extra_joints': {},
            'goals':[],
            'joint_limits':[],
            'joint_names':[],
            'joint_ordering':[],
            'joint_types':[],
            'mode_control':'absolute',
            'mode_environment':'ECAA',
            'nn_jointpoint':{'intercepts':[],'coefs':[],'split_point':None},
            'nn_main':{'intercepts':[],'coefs':[],'split_point':None},
            'objectives': [],
            'states': [],
            'robot_link_radius': 0.05,
            'rot_offsets': [],
            'starting_config': [],
            'urdf': '<?xml version="1.0" ?><robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"/><joint name="default_joint" type="fixed"><parent link="base_link" /><child link="default_link" /><origin xyz="0 0 0" rpy="0 0 0" /></joint><link name="default_link"/></robot>',
            'velocity_limits': [],
            'disp_offsets': [],
            'displacements': [],
            'links':[],
            'fixed_joints':[],
            'dynamic_joints':[],
            'displayed_state':[],
            'control':'manual'
        }
        self._update_order = ['urdf','fixed_frame','joint_names','ee_fixed_joints',
                              'joint_ordering','states','starting_config','robot_link_radius',
                              'static_environment','nn_jointpoint','nn_main','objectives',
                              'goals','base_link_motion_bounds','mode_control',
                              'mode_environment','config','solver']

        self._derivations = {
            'solver':[self.derive_control],
            'config':[self.derive_control,self.derive_solver],
            'robot':[self.derive_control,self.derive_axis_types,self.derive_joint_types,self.derive_joint_limits,
                     self.derive_rot_offsets,self.derive_velocity_limits,self.derive_disp_offsets,
                     self.derive_displacements,self.derive_nn],
            'parsed_urdf':[self.derive_control,self.derive_links,self.derive_dynamic_joints,self.derive_fixed_joints,
                           self.derive_extra_joints],
            'urdf':[self.derive_parsed_urdf],
            'fixed_frame':[self.derive_robot],
            'axis_types':[self.derive_config],
            'base_link_motion_bounds':[self.derive_config],
            'ee_fixed_joints':[self.derive_robot],
            'static_environment':[self.derive_nn],
            'goals':[self.derive_config],
            'joint_limits':[self.derive_starting_config],
            'joint_ordering':[self.derive_extra_joints],
            'joint_names':[self.derive_extra_joints],
            'joint_types':[self.derive_config],
            'mode_control':[self.derive_config],
            'mode_environment':[self.derive_config],
            'nn_jointpoint':[self.derive_config],
            'nn_main':[self.derive_config],
            'objectives':[self.derive_config],
            'states':[self.derive_nn],
            'robot_link_radius':{self.derive_nn},
            'rot_offsets':[self.derive_config],
            'starting_config':[self.derive_displayed_state,self.derive_config],
            'velocity_limits':[self.derive_config],
            'disp_offsets':[self.derive_config],
            'displacements':[self.derive_config],
            'links':[self.derive_fixed_frame],
            'dynamic_joints':[],
            'fixed_joints':[],
            'extra_joints':[self.derive_robot],
            'displayed_state':[],
            'control':[]
        }


        for field,value in self._defaults.items():
            setattr(self,'_'+field,value)

    def load(self,data):
        success = True
        for property in self._update_order:
            if property in data and success:
                try:
                    setattr(self,property,data[property])
                except:
                    success = False

    @property
    def simplified(self):
        shown = {key:value for key,value in self.data.items() if key != 'urdf'}
        if self.urdf == self._defaults['urdf']:
            shown['urdf'] = '<default urdf>'
        else:
            shown['urdf'] = '<custom urdf>'
        return shown

    # Macro for handling updates
    def handle_update(self,property,value):
        if getattr(self,'_'+property) != value:
            setattr(self,'_'+property,value)
            self.clear_dependencies(property)
            self.derive(property)

    def clear_dependencies(self,property):
        if property in self._dependencies:
            for dependency in self._dependencies[property]:
                setattr(self,dependency,self._defaults[dependency])

    def derive(self,property):
        if property not in self._derivations:
            print("DERIVATION NOT FOUND")
        for derivation in self._derivations[property]:
            derivation()

    @property
    def data(self):
        return dict(axis_types=self._axis_types,
                    base_link_motion_bounds=self._base_link_motion_bounds,
                    ee_fixed_joints=self._ee_fixed_joints,
                    static_environment=self._static_environment,
                    fixed_frame=self._fixed_frame,
                    goals=self._goals,
                    joint_limits=self._joint_limits,
                    joint_names=self._joint_names,
                    joint_ordering=self._joint_ordering,
                    joint_types=self._joint_types,
                    mode_control=self._mode_control,
                    mode_environment=self._mode_environment,
                    nn_jointpoint=self._nn_jointpoint,
                    nn_main=self._nn_main,
                    objectives=self._objectives,
                    states=self._states,
                    robot_link_radius=self._robot_link_radius,
                    rot_offsets=self._rot_offsets,
                    starting_config=self._starting_config,
                    urdf=self._urdf,
                    velocity_limits=self._velocity_limits,
                    disp_offsets=self._disp_offsets,
                    displacements=self._displacements)

    @property
    def meta(self):
        return {'valid_urdf': self.valid_urdf,
                'valid_robot': self.valid_robot,
                'valid_nn': self.valid_nn,
                'valid_config': self.valid_config,
                'valid_solver': self.valid_solver,
                'links': self.links,
                'dynamic_joints': self.dynamic_joints,
                'fixed_joints': self.fixed_joints,
                'displayed_state': self.displayed_state,
                'control': self.control}

    # Property setters, getters, and derivers

    @property
    def valid_urdf(self):
        return self.parsed_urdf != None

    @property
    def valid_robot(self):
        return self.robot != None

    @property
    def valid_nn(self):
        return self.nn_main != self._defaults['nn_main'] and self.nn_jointpoint != self._defaults['nn_jointpoint']

    @property
    def valid_config(self):
        return self.config != None

    @property
    def valid_solver(self):
        return self.solver != None

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self,value):
        self.handle_update('config',value)

    def derive_config(self):
        try:
            assert self.meta['valid_nn']
            self.config = parse_config_data(self.data)
        except:
            self.config = None

    @property
    def solver(self):
        return self._solver

    @solver.setter
    def solver(self,value):
        self.handle_update('solver',value)

    def derive_solver(self):
        try:
            assert self.meta['valid_config']
            self.solver = LivelyIK(self.config)
        except:
            self.solver = None

    @property
    def robot(self):
        return self._robot

    @robot.setter
    def robot(self,value):
        print('setter of robot {0}'.format(value))
        self.handle_update('robot',value)

    def derive_robot(self):
        try:
            arms = []
            for i in range(len(self.joint_names)):
                urdf_robot, arm, arm_c, tree = urdf_load_from_string(self.urdf, '', '', self.joint_names[i], self.ee_fixed_joints[i])
                arms.append(arm)

            self.robot = PythonRobot(arms, self.joint_names, self.joint_ordering, extra_joints=self.extra_joints)
        except:
            self.robot = None

    @property
    def parsed_urdf(self):
        return self._parsed_urdf

    @parsed_urdf.setter
    def parsed_urdf(self,value):
        self.handle_update('parsed_urdf',value)

    def derive_parsed_urdf(self):
        try:
            self.parsed_urdf = et.fromstring(self.urdf)
        except:
            self.parsed_urdf = None

    @property
    def urdf(self):
        return self._urdf

    @urdf.setter
    def urdf(self,value):
        self.handle_update('urdf',value.replace('\\',''))

    @property
    def axis_types(self):
        return self._axis_types

    @axis_types.setter
    def axis_types(self,value):
        self.handle_update('axis_types',value)

    def derive_axis_types(self):
        try:
            num_chains = self.robot.numChains
            axis_types = []
            for i in range(num_chains):
                arm_axes = []
                chain_len = len(self.robot.arms[i].axes)
                for j in range(chain_len):
                    arm_axes.append(self.robot.arms[i].axes[j])
                axis_types.append(arm_axes)
            self.axis_types = axis_types
        except:
            self.axis_types = self._defaults['axis_types']

    @property
    def base_link_motion_bounds(self):
        return self._base_link_motion_bounds

    @base_link_motion_bounds.setter
    def base_link_motion_bounds(self,value):
        self.handle_update('base_link_motion_bounds',value)

    @property
    def ee_fixed_joints(self):
        return self._ee_fixed_joints

    @ee_fixed_joints.setter
    def ee_fixed_joints(self,value):
        self.handle_update('ee_fixed_joints',value)

    @property
    def static_environment(self):
        return self._static_environment

    @static_environment.setter
    def static_environment(self,value):
        self.handle_update('static_environment',value)

    @property
    def fixed_frame(self):
        return self._fixed_frame

    @fixed_frame.setter
    def fixed_frame(self,value):
        self.handle_update('fixed_frame',value)

    def derive_fixed_frame(self):
        try:
            self.fixed_frame = self.links[0]
        except:
            self.fixed_frame = self._defaults['fixed_frame']

    @property
    def goals(self):
        return self._goals

    @goals.setter
    def goals(self,value):
        self.handle_update('goals',value)

    @property
    def joint_limits(self):
        return self._joint_limits

    @joint_limits.setter
    def joint_limits(self,value):
        self.handle_update('joint_limits',value)

    def derive_joint_limits(self):
        try:
            print(self.robot.bounds)
            self.joint_limits = [[pair[0],pair[1]] for pair in self.robot.bounds]
        except:
            self.joint_limits = self._defaults['joint_limits']

    @property
    def joint_names(self):
        return self._joint_names

    @joint_names.setter
    def joint_names(self,value):
        self.handle_update('joint_names',value)

    @property
    def joint_ordering(self):
        return self._joint_ordering

    @joint_ordering.setter
    def joint_ordering(self,value):
        self.handle_update('joint_ordering',value)

    @property
    def joint_types(self):
        return self._joint_types

    @joint_types.setter
    def joint_types(self,value):
        self.handle_update('joint_types',value)

    def derive_joint_types(self):
        try:
            num_chains = self.robot.numChains
            joint_types = []
            for i in range(num_chains):
                arm_types = []
                chain_len = len(self.robot.arms[i].joint_types)
                for j in range(chain_len):
                    arm_types.append(self.robot.arms[i].joint_types[j])
                joint_types.append(arm_types)
            self.joint_types = joint_types
        except:
            self.joint_types = self._defaults['joint_types']

    @property
    def mode_control(self):
        return self._mode_control

    @mode_control.setter
    def mode_control(self,value):
        self.handle_update('mode_control',value)

    @property
    def mode_environment(self):
        return self._mode_environment

    @mode_environment.setter
    def mode_environment(self,value):
        self.handle_update('mode_environment',value)

    @property
    def nn_jointpoint(self):
        return self._nn_jointpoint

    @nn_jointpoint.setter
    def nn_jointpoint(self,value):
        self.handle_update('nn_jointpoint',value)

    @property
    def nn_main(self):
        return self._nn_main

    @nn_main.setter
    def nn_main(self,value):
        self.handle_update('nn_main',value)

    def derive_nn(self):
        try:
            collision_graph = CollisionGraph(self.data, self.robot, sample_states=self._states)

            scores = []
            main_samples = []
            jointpoint_samples = []

            for i in range(200000):
                try:
                    state = self._states[i]
                except:
                    state = []
                    for b in self.robot.bounds:
                        rand = nprandom.uniform(low=b[0], high=b[1], size=(1))[0]
                        state.append(rand)
                main_samples.append(state)
                frames = robot.getFrames(state)
                jointpoint_samples.append(self.frames_to_jt_pt_vec(frames))
                scores.append(collision_graph.get_collision_score(frames))

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

            clf_main.fit(main_samples, scores)
            split_point = self.find_optimal_split_point(clf_main,self.robot,collision_graph,2000,jointpoint=False)

            self.nn_main = {'intercepts':clf_main.intercepts_,'coefs':clf_main.coefs_,'split_point':split_point}

            # Train JointPoint NN
            hls = []
            for i in range(num_layers):
                hls.append(layer_width)
            hls = tuple(hls)
            clf_jp = MLPRegressor(solver='adam', alpha=1,
                                    hidden_layer_sizes=hls, max_iter=300000, verbose=True,
                                    learning_rate='adaptive')

            clf_jp.fit(jointpoint_samples, scores)
            split_point = self.find_optimal_split_point(clf_jp,self.robot,collision_graph,2000,jointpoint=True)

            self.nn_jointpoint = {'intercepts':clf_jp.intercepts_,'coefs':clf_jp.coefs_,'split_point':split_point}
        except:
            self.nn_main = self._defaults['nn_main']
            self.nn_jointpoint = self._defaults['nn_jointpoint']

    @property
    def objectives(self):
        return self._objectives

    @objectives.setter
    def objectives(self,value):
        self.handle_update('objectives',value)

    @property
    def states(self):
        return self._states

    @states.setter
    def states(self,value):
        self.handle_update('states',value)

    @property
    def robot_link_radius(self):
        return self._robot_link_radius

    @robot_link_radius.setter
    def robot_link_radius(self,value):
        self.handle_update('robot_link_radius',value)

    @property
    def rot_offsets(self):
        return self._rot_offsets

    @rot_offsets.setter
    def rot_offsets(self,value):
        self.handle_update('rot_offsets',value)

    def derive_rot_offsets(self):
        try:
            num_chains = self.robot.numChains
            rot_offsets = []
            for i in range(num_chains):
                arm_offsets = []
                chain_len = len(self.robot.arms[i].original_rotOffsets)
                for j in range(chain_len):
                    d = self.robot.arms[i].original_rotOffsets[j]
                    arm_offsets.append([d[0], d[1], d[2]])
                rot_offsets.append(arm_offsets)
            self.rot_offsets = rot_offsets
        except:
            self.rot_offsets = self._defaults['rot_offsets']

    @property
    def starting_config(self):
        return self._starting_config

    @starting_config.setter
    def starting_config(self,value):
        self.handle_update('starting_config',value)

    def derive_starting_config(self):
        try:
            self.starting_config = [(limit[0]+limit[1])/2.0 for limit in self.joint_limits]
        except:
            self.starting_config = self._defaults['starting_config']

    @property
    def velocity_limits(self):
        return self._velocity_limits

    @velocity_limits.setter
    def velocity_limits(self,value):
        self.handle_update('velocity_limits',value)

    def derive_velocity_limits(self):
        try:
            self.velocity_limits = self.robot.velocity_limits
        except:
            self.velocity_limits = self._defaults['velocity_limits']

    @property
    def disp_offsets(self):
        return self._disp_offsets

    @disp_offsets.setter
    def disp_offsets(self,value):
        self.handle_update('disp_offsets',value)

    def derive_disp_offsets(self):
        try:
            num_chains = self.robot.numChains
            disp_offsets = []
            for i in range(num_chains):
                d = self.robot.arms[i].dispOffset
                disp_offsets.append([d[0], d[1], d[2]])
            self.disp_offsets = disp_offsets
        except:
            self.disp_offsets = self._defaults['disp_offsets']

    @property
    def displacements(self):
        return self._displacements

    @displacements.setter
    def displacements(self,value):
        self.handle_update('displacements',value)

    def derive_displacements(self):
        try:
            num_chains = self.robot.numChains
            displacements = []
            for i in range(num_chains):
                arm_displacements = []
                chain_len = len(self.robot.arms[i].displacements)
                for j in range(chain_len):
                    d = self.robot.arms[i].displacements[j]
                    arm_displacements.append([d[0], d[1], d[2]])
                displacements.append(arm_displacements)
            self.displacements = displacements
        except:
            self.displacements = self._defaults['displacements']

    @property
    def links(self):
        return self._links

    @links.setter
    def links(self,value):
        self.handle_update('links',value)

    def derive_links(self):
        try:
            links = []
            for child in self.parsed_urdf:
                if child.tag == 'link':
                    links.append(child.attrib['name'])
            self.links = links
        except:
            self.links = self._defaults['links']

    @property
    def dynamic_joints(self):
        return self._dynamic_joints

    @dynamic_joints.setter
    def dynamic_joints(self,value):
        self.handle_update('dynamic_joints',value)

    def derive_dynamic_joints(self):
        try:
            joints = []
            for child in self.parsed_urdf:
                if child.tag == 'joint' and child.attrib['type'] != 'fixed':
                    joints.append(child.attrib['name'])
            self.dynamic_joints = joints
        except:
            self.dynamic_joints = self._defaults['dynamic_joints']

    @property
    def fixed_joints(self):
        return self._fixed_joints

    @fixed_joints.setter
    def fixed_joints(self,value):
        self.handle_update('fixed_joints',value)

    def derive_fixed_joints(self):
        try:
            joints = []
            for child in self.parsed_urdf:
                if child.tag == 'joint' and child.attrib['type'] == 'fixed':
                    joints.append(child.attrib['name'])
            self.fixed_joints = joints
        except:
            self.fixed_joints = self._defaults['fixed_joints']

    @property
    def extra_joints(self):
        return self._extra_joints

    @extra_joints.setter
    def extra_joints(self,value):
        self.handle_update('extra_joints',value)

    def derive_extra_joints(self):
        try:
            extra_joint_names = []
            for joint in self.joint_ordering:
                found = False
                for chain in self.joint_names:
                    if joint in chain:
                        found = True
                if not found:
                    extra_joint_names.append(joint)
            extra_joints = {name:{'bounds':[0,0],'velocity':0} for name in extra_joint_names}
            for child in self.parsed_urdf:
                if child.tag == 'joint' and child.attrib['type'] != 'fixed' and child.attrib['name'] in extra_joint_names:
                    for attrib in child:
                        if attrib.tag == 'limit':
                            extra_joints[child.attrib['name']]['bounds'][0] = float(attrib.attrib['lower'])
                            extra_joints[child.attrib['name']]['bounds'][1] = float(attrib.attrib['upper'])
                            extra_joints[child.attrib['name']]['velocity'] = float(attrib.attrib['velocity'])
            self.extra_joints = extra_joints
        except:
            self.extra_joints = self._defaults['extra_joints']

    @property
    def displayed_state(self):
        return self._displayed_state

    @displayed_state.setter
    def displayed_state(self,value):
        self.handle_update('displayed_state',value)

    def derive_displayed_state(self):
        try:
            self.displayed_state = self.starting_config
        except:
            self.displayed_state = self._defaults['displayed_state']

    @property
    def control(self):
        return self._control

    @control.setter
    def control(self,value):
        self.handle_update('control',value)

    def derive_control(self):
        try:
            if self.valid_solver:
                print("VALID SOLVER FOUND")
                self.control = 'solve'
            else:
                print("NO VALID SOLVER")
                self.control = self._defaults['control']
        except:
            self.control = self._defaults['control']

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
