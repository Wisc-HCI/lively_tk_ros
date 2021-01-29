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

        self._fields = {
            'solver':{
                'default':None,
                'derivation':self.derive_solver,
                'derivations':['control'],
                'dependencies':[],
                'guards':[lambda:self.valid_config]
            },
            'config':{
                'default':None,
                'derivation':self.derive_config,
                'derivations':['control','solver'],
                'dependencies':[],
                'guards':[lambda:self.valid_nn]
            },
            'robot':{
                'default':None,
                'derivation':self.derive_robot,
                'derivations':['control','axis_types',
                               'joint_types','joint_limits',
                               'rot_offsets','velocity_limits',
                               'disp_offsets','displacements'],
                'dependencies':['objectives','goals'],
                'guards':[lambda:self.valid_urdf,lambda:self.valid_arms]
            },
            'parsed_urdf':{
                'default':None,
                'derivation':self.derive_parsed_urdf,
                'derivations':['control','links',
                               'dynamic_joints','fixed_joints',
                               'extra_joints'],
                'dependencies':['ee_fixed_joints','joint_names','joint_ordering','states'],
                'guards':[]
            },
            'urdf':{
                'default':'<?xml version="1.0" ?><robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"/><joint name="default_joint" type="fixed"><parent link="base_link" /><child link="default_link" /><origin xyz="0 0 0" rpy="0 0 0" /></joint><link name="default_link"/></robot>',
                'derivation':lambda:self.derive_from_default('robot_link_radius'),
                'derivations':['parsed_urdf'],
                'dependencies':['base_link_motion_bounds','mode_control','mode_environment',
                                'robot_link_radius','static_environment'],
                'guards':[]
            },
            'axis_types':{
                'default':[],
                'derivation':self.derive_axis_types,
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'base_link_motion_bounds':{
                'default':[[0,0],[0,0],[0,0]],
                'derivation':lambda:self.derive_from_default('base_link_motion_bounds'),
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_urdf]
            },
            'fixed_frame':{
                'default':'base_link',
                'derivation':self.derive_fixed_frame,
                'derivations':['robot'],
                'dependencies':[],
                'guards':[lambda:self.valid_urdf,lambda:len(self.links)>0]
            },
            'ee_fixed_joints':{
                'default':[],
                'derivation':lambda:self.derive_from_default('ee_fixed_joints'),
                'derivations':['robot'],
                'dependencies':[],
                'guards':[lambda:self.valid_urdf]
            },
            'static_environment':{
                'default':{
                        'cuboids':[],
                        'spheres':[],
                        'pcs':[]
                    },
                'derivation':lambda:self.derive_from_default('static_environment'),
                'derivations':[],
                'dependencies':[],
                'guards':[lambda:self.valid_urdf]
            },
            'extra_joints':{
                'default':{},
                'derivation':self.derive_extra_joints,
                'derivations':['robot'],
                'dependencies':[],
                'guards':[lambda:self.valid_urdf]
            },
            'goals':{
                'default':[],
                'derivation':lambda:self.derive_from_default('goals'),
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'joint_limits':{
                'default':[],
                'derivation':self.derive_joint_limits,
                'derivations':['starting_config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'joint_ordering':{
                'default':[],
                'derivation':lambda:self.derive_from_default('joint_ordering'),
                'derivations':['extra_joints'],
                'dependencies':[],
                'guards':[lambda:self.valid_urdf]
            },
            'joint_names':{
                'default':[],
                'derivation':lambda:self.derive_from_default('joint_names'),
                'derivations':['extra_joints'],
                'dependencies':[],
                'guards':[lambda:self.valid_urdf]
            },
            'joint_types':{
                'default':[],
                'derivation':self.derive_joint_types,
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'mode_control':{
                'default':'absolute',
                'derivation':lambda:self.derive_from_default('mode_control'),
                'derivations':['config'],
                'dependencies':[],
                'guards':[]
            },
            'mode_environment':{
                'default':'ECAA',
                'derivation':lambda:self.derive_from_default('mode_environment'),
                'derivations':['config'],
                'dependencies':[],
                'guards':[]
            },
            'nn_jointpoint':{
                'default':{'intercepts':[],'coefs':[],'split_point':None},
                'derivation':lambda:self.derive_from_default('nn_jointpoint'),
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'nn_main':{
                'default':{'intercepts':[],'coefs':[],'split_point':None},
                'derivation':lambda:self.derive_from_default('nn_main'),
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'objectives':{
                'default':[],
                'derivation':lambda:self.derive_from_default('objectives'),
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'states':{
                'default':[],
                'derivation':lambda:self.derive_from_default('states'),
                'derivations':[],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'robot_link_radius':{
                'default':0.05,
                'derivation':lambda:self.derive_from_default('robot_link_radius'),
                'derivations':[],
                'dependencies':[],
                'guards':[]
            },
            'rot_offsets':{
                'default':[],
                'derivation':self.derive_rot_offsets,
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'starting_config':{
                'default':[],
                'derivation':self.derive_starting_config,
                'derivations':['displayed_state','config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'velocity_limits':{
                'default':[],
                'derivation':self.derive_velocity_limits,
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'disp_offsets':{
                'default':[],
                'derivation':self.derive_disp_offsets,
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'displacements':{
                'default':[],
                'derivation':self.derive_displacements,
                'derivations':['config'],
                'dependencies':[],
                'guards':[lambda:self.valid_robot]
            },
            'links':{
                'default':[],
                'derivation':self.derive_links,
                'derivations':['fixed_frame'],
                'dependencies':[],
                'guards':[]
            },
            'fixed_joints':{
                'default':[],
                'derivation':self.derive_fixed_joints,
                'derivations':[],
                'dependencies':[],
                'guards':[]
            },
            'dynamic_joints':{
                'default':[],
                'derivation':self.derive_dynamic_joints,
                'derivations':[],
                'dependencies':[],
                'guards':[]
            },
            'displayed_state':{
                'default':[],
                'derivation':self.derive_displayed_state,
                'derivations':[],
                'dependencies':[],
                'guards':[]
            },
            'control':{
                'default':'manual',
                'derivation':self.derive_control,
                'derivations':[],
                'dependencies':[],
                'guards':[]
            }
        }


        self._update_order = ['urdf','fixed_frame','joint_names','ee_fixed_joints',
                              'joint_ordering','states','starting_config','robot_link_radius',
                              'static_environment','nn_jointpoint','nn_main','objectives',
                              'goals','base_link_motion_bounds','mode_control',
                              'mode_environment','config','solver']

        for field,value in self._fields.items():
            setattr(self,'_'+field,value['default'])

    def load(self,data):
        for property in self._update_order:
            if property in data:
                print("SETTING {0}".format(property))
                setattr(self,property,data[property])

    def derive_from_default(self,field):
        setattr(self,field,self._fields[field]['default'])

    @property
    def simplified(self):
        shown = {key:value for key,value in self.data.items() if key != 'urdf'}
        if self.urdf == self._fields['urdf']['default']:
            shown['urdf'] = '<default urdf>'
        else:
            shown['urdf'] = '<custom urdf>'
        return shown

    # Macro for handling updates
    def handle_update(self,property,value):
        print('handle update of {0} to {1}'.format(property, value))
        if getattr(self,'_'+property) != value:
            # Prevent the property from being set if
            # any of the guards fail
            for i,guard in enumerate(self._fields[property]['guards']):
                if not guard():
                    print('Prevented setting {0} by guard {1}'.format(property,i))
                    setattr(self,'_'+property,self._fields[property]['default'])
                    self.clear_dependencies(property)
                    return
            setattr(self,'_'+property,value)
            self.clear_dependencies(property)
            self.derive(property)

    def clear_dependencies(self,property):
        for dependency in self._fields[property]['dependencies']:
            setattr(self,dependency,self._fields[dependency]['default'])

    def derive(self,property):
        for derivation in self._fields[property]['derivations']:
            # Prevent the derivation from being set if
            # any of the guards fail
            for i,guard in enumerate(self._fields[derivation]['guards']):
                if not guard():
                    print('Prevented deriving {0} by guard {1}'.format(derivation,i))
                    return
            print('Deriving {0} after updating {1}'.format(derivation,property))
            self._fields[derivation]['derivation']()

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
    def valid_arms(self):
        if len(self.joint_names) == len(self.ee_fixed_joints) and len(self.joint_names) > 0 and len(self.joint_names) > 0:
            for chain in self.joint_names:
                for joint in chain:
                    if joint not in self.joint_ordering:
                        return False
            return True
        else:
            return False

    @property
    def valid_robot(self):
        return self.robot != None and self.valid_urdf

    @property
    def valid_nn(self):
        return self.valid_robot and self.nn_main != self._fields['nn_main']['default'] and self.nn_jointpoint != self._fields['nn_jointpoint']['default']

    @property
    def valid_config(self):
        return self.valid_nn and self.config != None

    @property
    def valid_solver(self):
        return self.valid_config and self.solver != None

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self,value):
        self.handle_update('config',value)

    def derive_config(self):
        self.config = parse_config_data(self.data)

    @property
    def solver(self):
        return self._solver

    @solver.setter
    def solver(self,value):
        self.handle_update('solver',value)

    def derive_solver(self):
        self.solver = LivelyIK(self.config)

    @property
    def robot(self):
        return self._robot

    @robot.setter
    def robot(self,value):
        print('setter of robot {0}'.format(value))
        self.handle_update('robot',value)

    def derive_robot(self):
        print(self.joint_names,self.joint_ordering,self.ee_fixed_joints,self.extra_joints)
        print(self.valid_arms)
        arms = []
        for i in range(len(self.joint_names)):
            urdf_robot, arm, arm_c, tree = urdf_load_from_string(self.urdf, '', '', self.joint_names[i], self.ee_fixed_joints[i])
            arms.append(arm)

        self.robot = PythonRobot(arms, self.joint_names, self.joint_ordering, extra_joints=self.extra_joints)

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
        num_chains = self.robot.numChains
        axis_types = []
        for i in range(num_chains):
            arm_axes = []
            chain_len = len(self.robot.arms[i].axes)
            for j in range(chain_len):
                arm_axes.append(self.robot.arms[i].axes[j])
            axis_types.append(arm_axes)
        self.axis_types = axis_types

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
        self.fixed_frame = self.links[0]

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
        self.joint_limits = [[pair[0],pair[1]] for pair in self.robot.bounds]

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
        num_chains = self.robot.numChains
        joint_types = []
        for i in range(num_chains):
            arm_types = []
            chain_len = len(self.robot.arms[i].joint_types)
            for j in range(chain_len):
                arm_types.append(self.robot.arms[i].joint_types[j])
            joint_types.append(arm_types)
        self.joint_types = joint_types

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
            frames = self.robot.getFrames(state)
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

    @property
    def starting_config(self):
        return self._starting_config

    @starting_config.setter
    def starting_config(self,value):
        self.handle_update('starting_config',value)

    def derive_starting_config(self):
        self.starting_config = [(limit[0]+limit[1])/2.0 for limit in self.joint_limits]

    @property
    def velocity_limits(self):
        return self._velocity_limits

    @velocity_limits.setter
    def velocity_limits(self,value):
        self.handle_update('velocity_limits',value)

    def derive_velocity_limits(self):
        self.velocity_limits = self.robot.velocity_limits

    @property
    def disp_offsets(self):
        return self._disp_offsets

    @disp_offsets.setter
    def disp_offsets(self,value):
        self.handle_update('disp_offsets',value)

    def derive_disp_offsets(self):
        num_chains = self.robot.numChains
        disp_offsets = []
        for i in range(num_chains):
            d = self.robot.arms[i].dispOffset
            disp_offsets.append([d[0], d[1], d[2]])
        self.disp_offsets = disp_offsets

    @property
    def displacements(self):
        return self._displacements

    @displacements.setter
    def displacements(self,value):
        self.handle_update('displacements',value)

    def derive_displacements(self):
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

    @property
    def links(self):
        return self._links

    @links.setter
    def links(self,value):
        self.handle_update('links',value)

    def derive_links(self):
        links = []
        for child in self.parsed_urdf:
            if child.tag == 'link':
                links.append(child.attrib['name'])
        self.links = links

    @property
    def dynamic_joints(self):
        return self._dynamic_joints

    @dynamic_joints.setter
    def dynamic_joints(self,value):
        self.handle_update('dynamic_joints',value)

    def derive_dynamic_joints(self):
        joints = []
        for child in self.parsed_urdf:
            if child.tag == 'joint' and child.attrib['type'] != 'fixed':
                joints.append(child.attrib['name'])
        self.dynamic_joints = joints

    @property
    def fixed_joints(self):
        return self._fixed_joints

    @fixed_joints.setter
    def fixed_joints(self,value):
        self.handle_update('fixed_joints',value)

    def derive_fixed_joints(self):
        joints = []
        for child in self.parsed_urdf:
            if child.tag == 'joint' and child.attrib['type'] == 'fixed':
                joints.append(child.attrib['name'])
        self.fixed_joints = joints

    @property
    def extra_joints(self):
        return self._extra_joints

    @extra_joints.setter
    def extra_joints(self,value):
        self.handle_update('extra_joints',value)

    def derive_extra_joints(self):
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

    @property
    def displayed_state(self):
        return self._displayed_state

    @displayed_state.setter
    def displayed_state(self,value):
        self.handle_update('displayed_state',value)

    def derive_displayed_state(self):
        self.displayed_state = self.starting_config

    @property
    def control(self):
        return self._control

    @control.setter
    def control(self,value):
        self.handle_update('control',value)

    def derive_control(self):
        if self.valid_solver:
            self.control = 'solve'
        else:
            self.control = self._fields['control']['default']

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
