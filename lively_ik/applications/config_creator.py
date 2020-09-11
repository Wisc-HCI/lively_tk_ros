import eventlet
eventlet.monkey_patch()
from threading import Thread
from lively_ik.applications.app import App
from lively_ik.utils.urdf_load import urdf_load_from_string
from lively_ik.spacetime.robot import Robot
from lively_ik import BASE, SRC
import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import inflection
import xml.etree.ElementTree as et
import yaml
from julia import LivelyIK
from flask_socketio import emit
import asyncio
import os

class ConfigCreator(App):

    def __init__(self, node):
        super(ConfigCreator,self).__init__(node,'Config Creator','config_creator')
        self.js_pub = self.node.create_publisher(JointState,'/joint_states',10)
        self.tf_pub = self.node.create_publisher(TFMessage,'tf_static',10)
        self.js_timer = self.node.create_timer(0.05,self.publish_joint_states)
        self.tf_timer = self.node.create_timer(1,self.publish_tf)
        self.robot_description_client = self.node.create_client(SetParameters, '/robot_state_publisher/set_parameters')
        self.robot_setup = False
        self.is_preprocessing = False
        self.preprocessing_steps = ['write_yaml','julia_nn','julia_params','python']
        self.preprocessing_thread = None

        self.robot=None
        self.xml_tree=None
        self.container=None

        # App State Info
        self.step = 0
        self.displayed_state = [] # Published to /joint_states
        self.preprocessing_state = {step:{'progress':0.0,'ok':True} for step in self.preprocessing_steps}

        # Config Contents
        self.urdf=None               # User-provided
        self.robot_name=None         # User-provided
        self.fixed_frame='base_link' # User-provided
        self.joint_ordering=[]       # User-provided
        self.joint_names=[]          # User-provided
        self.ee_fixed_joints=[]      # User-provided
        self.starting_config=[]      # User-provided
        self.objectives=[]           # User-provided
        self.fixed_frame_noise_scale=0     # User-provided
        self.fixed_frame_noise_frequency=0 # User-provided
        self.mode='absolute'               # User-provided
        self.axis_types=[]           # Computed
        self.disp_offsets=[]         # Computed
        self.rot_offsets=[]          # Computed
        self.joint_types=[]          # Computed
        self.all_links=[]            # Computed
        self.all_dynamic_joints=[]   # Computed
        self.all_fixed_joints=[]     # Computed
        self.displacements=[]        # Computed


        # Properties: joint_limits, velocity_limits

        # Collision Contents
        self.robot_link_radius=0.05  # User-provided
        self.sample_states=[]        # User-provided
        self.training_states=[]      # User-provided
        self.problem_states=[]       # User-provided
        self.boxes=[]                # User-provided
        self.spheres=[]              # User-provided
        self.ellipsoids=[]           # User-provided
        self.capsules=[]             # User-provided
        self.cylinders=[]            # User-provided

        # Utility
        self.requires_robot_update = set(('urdf','jointNames','jointOrdering','eeFixedJoints'))
        self.requires_preprocessing = self.requires_robot_update.union(set(('robotLinkRadius','sampleStates','trainingStates','problemStates',
                                                                            'boxes','spheres','ellipsoids','capsules','cylinders','robotName')))

    @property
    def json_app(self):
        return {'step':self.step,'canStep':self.can_step,
                'displayedState':self.displayed_state,
                'preprocessingState':self.preprocessing_state}

    @property
    def config(self):
        return {
            'all_dynamic_joints':self.all_dynamic_joints,
            'all_fixed_joints':self.all_fixed_joints,
            'all_links':self.all_links,
            'axis_types':self.axis_types,
            'boxes':self.boxes,
            'capsules':self.capsules,
            'cylinders':self.cylinders,
            'displacements':self.displacements,
            'disp_offsets':self.disp_offsets,
            'ee_fixed_joints':self.ee_fixed_joints,
            'ellipsoids':self.ellipsoids,
            'fixed_frame':self.fixed_frame,
            'fixed_frame_noise_scale':self.fixed_frame_noise_scale,
            'fixed_frame_noise_frequency':self.fixed_frame_noise_frequency,
            'joint_limits':self.joint_limits,
            'joint_names':self.joint_names,
            'joint_ordering':self.joint_ordering,
            'joint_types':self.joint_types,
            'mode':self.mode,
            'objectives':self.objectives,
            'problem_states':self.problem_states,
            'robot_link_radius':self.robot_link_radius,
            'robot_name':self.robot_name,
            'rot_offsets':self.rot_offsets,
            'sample_states':self.sample_states,
            'spheres':self.spheres,
            'starting_config':self.starting_config,
            'training_states':self.training_states,
            'urdf':self.urdf,
            'velocity_limits':self.velocity_limits
        }

    @property
    def yaml(self):
        return yaml.dump(self.config)

    def publish_joint_states(self):
        if self.robot and self.robot_setup and len(self.joint_ordering) == len(self.displayed_state):
            js_msg = JointState(name=self.joint_ordering,position=self.displayed_state)
            js_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.js_pub.publish(js_msg)

    def publish_tf(self):
        tf1 = TransformStamped(header=Header(stamp=self.node.get_clock().now().to_msg(),
                                             frame_id='1'),
                               child_frame_id='world',
                               transform=Transform(translation=Vector3(x=0.0,y=0.0,z=0.0),
                                                   rotation=Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)))
        tf2 = TransformStamped(header=Header(stamp=self.node.get_clock().now().to_msg(),
                                             frame_id='world'),
                               child_frame_id=self.fixed_frame,
                               transform=Transform(translation=Vector3(x=0.0,y=0.0,z=0.0),
                                                   rotation=Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)))

        tf_msg = TFMessage(transforms=[tf1,tf2])
        self.tf_pub.publish(tf_msg)

    def preprocess_cb(self,key,progress):
        p = round(progress)
        if self.preprocessing_state[key]['progress'] != p:
            self.log('CB: {0} - {1}'.format(key,p))
            self.preprocessing_state[key]['progress'] = p
        #emit('app_process_response',{'success':True,'action':'preprocess','app':self.json_app})

    def write_config(self, cb):
        cb(0.0)
        with open(BASE+'/config/info_files/'+self.robot_name+'.yaml','w') as base_save:
            yaml.dump(self.config,base_save)
        cb(50.0)
        os.symlink(BASE+'/config/info_files/'+self.robot_name+'.yaml',SRC+'/config/info_files/'+self.robot_name+'.yaml')
        cb(100.0)

    def preprocess(self):
        self.is_preprocessing = True
        success = True
        for step in self.preprocessing_steps:
            if self.preprocessing_state[step]['progress'] < 100.0:
                self.log('Start: {0}'.format(step))
                #try:
                if step == 'writing_yaml':
                    self.write_config(lambda progress: self.preprocess_cb(step,progress))
                elif step == 'julia_nn':
                    LivelyIK.preprocess_phase1(self.yaml,self.node,lambda progress: self.preprocess_cb(step,progress))
                    # Create a symlink between LivelyIK SRC and BASE
                    for nn_suffix in ['_1','_2','_3']:
                        os.symlink(BASE+"/config/collision_nn/"+self.robot_name+nn_suffix,SRC+"/config/collision_nn/"+self.robot_name+nn_suffix)
                elif step == 'julia_params':
                    LivelyIK.preprocess_phase2(self.yaml,self.node,lambda progress: self.preprocess_cb(step,progress))
                    # Create a symlink between LivelyIK SRC and BASE
                    for nn_suffix in ['_params_1','_params_2','_params_3','_network_rank']:
                        os.symlink(BASE+"/config/collision_nn/"+self.robot_name+nn_suffix,SRC+"/config/collision_nn/"+self.robot_name+nn_suffix)
                elif step == 'python':
                    # TODO: Implement python preprocesssing
                    self.preprocessing_state[step]['progress'] = 100.0
                self.preprocessing_state[step]['ok'] = True
                # except Exception as e:
                #     self.preprocessing_state[step]['ok'] = False
                #     self.error('{0} error: {1}'.format(step,e))
                #     success = False
                #     break
        self.is_preprocessing = False
        emit('app_process_response',{'success':success,'action':'preprocess','app':self.json_app})



    @property
    def json_config(self):
        return {inflection.camelize(key,False):value for key,value in self.config.items()}

    def update(self,data):
        if data['action'] == 'config_update':
            return self.update_config(data)
        elif data['action'] == 'fetch':
            return {'success':True,'action':'fetch','config':self.json_config,'app':self.json_app}
        elif data['action'] == 'can_step':
            return {'success':True,'action':'can_step','app':self.json_app}
        elif data['action'] == 'step':
            return self.perform_step(data['direction'])

    def process(self,data):
        if data['action'] == 'preprocess' and not self.is_preprocessing:
            self.log('starting preprocessing')
            self.is_preprocessing = True
            self.preprocessing_thread = Thread(target=self.preprocess,daemon=True)
            self.preprocessing_thread.start()
            #asyncio.run(self.preprocess())
            # self.preprocess()
            while self.is_preprocessing:
                success = self.preprocessing_state['write_yaml']['ok'] and self.preprocessing_state['julia_nn']['ok'] and self.preprocessing_state['julia_params']['ok'] and self.preprocessing_state['python']['ok']
                #self.log('Process: {0}'.format(self.preprocessing_state))
                app = self.json_app
                yield {'success':success,'action':'preprocess','app':app}
            self.preprocessing_thread.join()
            self.preprocessing_thread = None


    def perform_step(self,direction):
        success = True
        if direction == 'forward':
            if self.can_step:
                self.step += 1
            else:
                success = False
        else:
            self.step -= 1
        return {'success':success,'action':'step','app':self.json_app}

    def update_config(self,data):
        config = data['config']
        if len(self.requires_robot_update.intersection(set(config.keys()))) > 0:
            changed = False;
            for key in [k for k in self.requires_robot_update if k in config.keys()]:
                prop = inflection.underscore(key)
                if key == 'urdf' and config[key] != getattr(self,prop):
                    changed = True
                    self.log("URDF SHOULD BE UPDATED")
                    setattr(self,prop,config[key])
                    if self.urdf != None:
                        self.log("NON-NULL URDF UPDATED")
                        self.parse_urdf()
                        break
                elif config[key] != getattr(self,prop):
                    changed = True
                    setattr(self,prop,config[key])
            if changed and self.urdf != None and self.has_parsable_urdf:
                try:
                    self.create_robot()
                except Exception as e:
                    self.robot = None
                    self.warn(str(e))
        self.robot_name = config.get('robotName',self.robot_name)
        self.fixed_frame = config.get('fixedFrame',self.fixed_frame)
        self.starting_config = config.get('startingConfig',self.starting_config)
        self.objectives = config.get('objectives',self.objectives)
        self.boxes = config.get('boxes',self.boxes)
        self.spheres = config.get('spheres',self.spheres)
        self.ellipsoids = config.get('ellipsoids',self.ellipsoids)
        self.capsules = config.get('capsules',self.capsules)
        self.cylinders = config.get('cylinders',self.cylinders)
        self.sample_states = config.get('sampleStates',self.sample_states)
        self.training_states = config.get('trainingStates',self.training_states)
        self.problem_states = config.get('problemStates',self.problem_states)
        self.fixed_frame_noise_scale = config.get('fixedFrameNoiseScale',self.fixed_frame_noise_scale)
        self.fixed_frame_noise_frequency = config.get('fixedFrameNoiseFrequency',self.fixed_frame_noise_frequency)
        self.mode = config.get('mode',self.mode)
        self.robot_link_radius = config.get('robotLinkRadius',self.robot_link_radius)

        if 'app' in data:
            if 'step' in data['app']:
                self.step = data['app']['step']
            if 'displayedState' in data['app']:
                self.displayed_state = data['app']['displayedState']

        return {'success':True,'action':'config_update','config':self.json_config,'app':self.json_app}


    @property
    def joint_limits(self):
        if self.robot:
            return [[pair[0],pair[1]] for pair in self.robot.bounds]
        else:
            return []

    @property
    def velocity_limits(self):
        if self.robot:
            return self.robot.velocity_limits
        else:
            return []

    @property
    def has_parsable_urdf(self):
        try:
            et.fromstring(self.urdf)
            return True
        except:
            return True

    def parse_urdf(self):
        if self.has_parsable_urdf:
            robot_root = et.fromstring(self.urdf)
            self.all_links = []
            self.all_fixed_joints = []
            self.all_dynamic_joints = []
            self.joint_ordering = []
            self.joint_names = []
            if 'name' in robot_root.attrib:
                self.robot_name = robot_root.attrib['name']
            for child in robot_root:
                if child.tag == 'link':
                    self.all_links.append(child.attrib['name'])
                elif child.tag == 'joint' and child.attrib['type'] == 'fixed':
                    self.all_fixed_joints.append(child.attrib['name'])
                elif child.tag == 'joint' and child.attrib['type'] != 'fixed':
                    self.all_dynamic_joints.append(child.attrib['name'])


    def create_robot(self):
        self.robot_setup = False
        arms = []
        for i in range(len(self.joint_names)):
            urdf_robot, arm, arm_c, tree = urdf_load_from_string(self.urdf, '', '', self.joint_names[i], self.ee_fixed_joints[i])
            arms.append(arm)

        # make robot
        self.robot = Robot(arms, self.joint_names, self.joint_ordering, extra_joints=self.extra_joints)
        self.log(str(self.joint_limits))
        self.starting_config = [(limit[0]+limit[1])/2.0 for limit in self.joint_limits]
        self.displayed_state = self.starting_config

        num_chains = self.robot.numChains

        axis_types = []
        for i in range(num_chains):
            arm_axes = []
            chain_len = len(self.robot.arms[i].axes)
            for j in range(chain_len):
                arm_axes.append(self.robot.arms[i].axes[j])
            axis_types.append(arm_axes)
        self.axis_types = axis_types

        displacements = []
        for i in range(num_chains):
            arm_displacements = []
            chain_len = len(self.robot.arms[i].displacements)
            for j in range(chain_len):
                d = self.robot.arms[i].displacements[j]
                arm_displacements.append([d[0], d[1], d[2]])
            displacements.append(arm_displacements)
        self.displacements = displacements


        disp_offsets = []
        for i in range(num_chains):
            d = self.robot.arms[i].dispOffset
            disp_offsets.append([d[0], d[1], d[2]])
        self.disp_offsets = disp_offsets

        rot_offsets = []
        for i in range(num_chains):
            arm_offsets = []
            chain_len = len(self.robot.arms[i].original_rotOffsets)
            for j in range(chain_len):
                d = self.robot.arms[i].original_rotOffsets[j]
                arm_offsets.append([d[0], d[1], d[2]])
            rot_offsets.append(arm_offsets)
        self.rot_offsets = rot_offsets

        joint_types = []
        for i in range(num_chains):
            arm_types = []
            chain_len = len(self.robot.arms[i].joint_types)
            for j in range(chain_len):
                arm_types.append(self.robot.arms[i].joint_types[j])
            joint_types.append(arm_types)
        self.joint_types = joint_types

        # Setup so the robot appears in RVIZ
        self.robot_setup = self.update_robot_description()


    @property
    def can_step(self):
        if self.step == 0:
            return self.urdf != None and self.robot_name != None and self.fixed_frame != None
        elif self.step == 1:
            if len(self.joint_ordering) == 0:
                return False
            if len(self.joint_names) == 0:
                return False
            if len(self.ee_fixed_joints) != len(self.joint_names):
                return False
            for chain in self.joint_names:
                for joint in chain:
                    if joint not in self.joint_ordering:
                        return False
            if not self.robot:
                return False
            return True
        elif self.step == 2:
            if not self.robot_setup:
                return False
            if len(self.joint_ordering) != len(self.starting_config):
                return False
            for idx, limits in enumerate(self.joint_limits):
                if limits[0] > self.starting_config[idx] or limits[1] < self.starting_config[idx]:
                    return False
            return True
        elif self.step == 7:
            complete = True
            for step in self.preprocessing_steps:
                if not self.preprocessing_state[step]['progress'] >= 100 or not self.preprocessing_state[step]['ok']:
                    complete = False
            if self.is_preprocessing or complete:
                return False
            return True
        else:
            return True

    def update_robot_description(self):
        request = SetParameters.Request()
        request.parameters = [Parameter(name='robot_description',value=ParameterValue(type=4,string_value=self.urdf))]
        future = self.robot_description_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        return response.results[0].successful

    @property
    def extra_joints(self):
        extra_joint_names = []
        for joint in self.joint_ordering:
            found = False
            for chain in self.joint_names:
                if joint in chain:
                    found = True
            if not found:
                extra_joint_names.append(joint)
        extra_joints = {name:{'bounds':[0,0],'velocity':0} for name in extra_joint_names}
        robot_root = et.fromstring(self.urdf)
        for child in robot_root:
            if child.tag == 'joint' and child.attrib['type'] != 'fixed' and child.attrib['name'] in extra_joint_names:
                for attrib in child:
                    if attrib.tag == 'limit':
                        extra_joints[child.attrib['name']]['bounds'][0] = float(attrib.attrib['lower'])
                        extra_joints[child.attrib['name']]['bounds'][1] = float(attrib.attrib['upper'])
                        extra_joints[child.attrib['name']]['velocity'] = float(attrib.attrib['velocity'])
        return extra_joints
