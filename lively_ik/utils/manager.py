import yaml
from julia import LivelyIK, Rotations
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarker, InteractiveMarkerControl, Marker
from pyquaternion import Quaternion as pyQuaternion
import json
from copy import deepcopy
import random

JOINT_SCALES = {'ur5_robotiq_85':{'elbow_joint': 0.04943172615553592,
                                  'shoulder_lift_joint': 0.06519351163038933,
                                  'shoulder_pan_joint': 0.05981500545991754,
                                  'wrist_1_joint': 0.060635328273466516,
                                  'wrist_2_joint': 0.03854340782800793,
                                  'wrist_3_joint': 0.08667862514476901,
                                  'gripper_finger1_joint':0,
                                  'gripper_finger2_joint':0
                },
                'panda':{'panda_joint1':0,
                         'panda_joint2':0,
                         'panda_joint3':0,
                         'panda_joint4':0,
                         'panda_joint5':0,
                         'panda_joint6':0,
                         'panda_joint7':0,
                         'panda_finger_joint1':0,
                         'panda_finger_joint2':0
                },
                'nao_v5':{'HeadPitch': 0.03241025901778748,
                         'HeadYaw': 0.03487730221459542,
                         'LAnklePitch': 0.01959773277218994,
                         'LAnkleRoll': 0.017679353837950794,
                         'LElbowRoll': 0.006171122449291864,
                         'LElbowYaw': 0.07014936768535909,
                         'LHand': 0.06780014567033442,
                         'LHipPitch': 0.019179325881739477,
                         'LHipRoll': 0.03467096850855242,
                         'LHipYawPitch': 0.035268050925749216,
                         'LKneePitch': 0.0039363565040527355,
                         'LShoulderPitch': 0.01499393991605014,
                         'LShoulderRoll': 0.012639605328994576,
                         'LWristYaw': 0.06573754145897988,
                         'RAnklePitch': 0.02066244887363038,
                         'RAnkleRoll': 0.01746599550689567,
                         'RElbowRoll': 0.01501447245298513,
                         'RElbowYaw': 0.09027797095715812,
                         'RHand': 0.0704930839703507,
                         'RHipPitch': 0.021021206596846108,
                         'RHipRoll': 0.03479924641126292,
                         'RHipYawPitch': 0.035266210267831555,
                         'RKneePitch': 0.014729152274376534,
                         'RShoulderPitch': 0.019397132143749468,
                         'RShoulderRoll': 0.017491677632176367,
                         'RWristYaw': 0.07781985791058026
                }
        }

ALLOWED_NAIVE_JOINTS = {
                    'ur5_robotiq_85':['elbow_joint','shoulder_lift_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint','gripper_finger1_joint','gripper_finger2_joint'],
                    'panda':['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7','panda_finger_joint1','panda_finger_joint2'],
                    'nao_v5':['HeadPitch','HeadYaw','LElbowRoll','LElbowYaw','LHand','LShoulderPitch','LShoulderRoll','LWristYaw','RElbowRoll','RElbowYaw','RHand','RShoulderPitch','RShoulderRoll','RWristYaw']
                    }

class Goal(object):
    def __init__(self,goal_value,goal_time):
        self.goal_value = goal_value
        self.goal_time = goal_time
        self.initial_time = None
        self.initial_value = None

    def increment_towards(self,current_value,current_time):
        if self.initial_time == None:
            self.initial_time = current_time
        if self.initial_value == None:
            self.initial_value = current_value
        if self.goal_time <= current_time:
            return self.goal_value
        else:
            proportion_of_time = (current_time - self.initial_time)/(self.goal_time - self.initial_time)
            # print(proportion_of_time)
            if type(self.goal_value) == float or type(self.goal_value) == int:
                return self.initial_value + proportion_of_time * (self.goal_value + self.initial_value)
            elif type(self.goal_value) == Rotations.Quat:
                goal_quat = pyQuaternion(w=self.goal_value.w,x=self.goal_value.x,y=self.goal_value.y,z=self.goal_value.z)
                initial_quat = pyQuaternion(w=self.initial_value.w,x=self.initial_value.x,y=self.initial_value.y,z=self.initial_value.z)
                current = pyQuaternion.slerp(initial_quat,goal_quat,proportion_of_time)
                return Rotations.Quat(w=current.w,x=current.x,y=current.y,z=current.z)
            elif self.goal_value == None:
                return current_value

    def update_goal(self,goal_value,goal_time):
        self.goal_value = goal_value
        self.goal_time = goal_time
        self.initial_time = None
        self.initial_value = None

class Manager(object):
    def __init__(self,node,info):
        self.node = node
        self.info = info
        self.ims = InteractiveMarkerServer(self,'manager')
        self.rik_container = RelaxedIKContainer(self.info)
        self.out_contents = ''
        self.out_file = None
        self.collecting = False
        self.conditions = []
        ee_positions = self.rik_container.robot.get_ee_positions(self.info['starting_config'])
        ee_rotations = self.rik_container.robot.get_ee_rotations(self.info['starting_config'])
        self.marker_lookup = []

        self.current_goal = {
            'positions':[],
            'rotations':[],
            'dc':[],
            'bias':[1,1,1],
            'weights':[]
        }

        self.future_goal = {
            'positions':[],
            'rotations':[],
            'dc':[],
            'bias':[Goal(1,0),Goal(1,0),Goal(1,0)],
            'weights':[]
        }

        for ee_idx in range(len(self.info['joint_names'])):
            # Determine initial pose info for goals
            self.current_goal['positions'].append(ee_positions[ee_idx][0:3])
            self.current_goal['rotations'].append(Rotations.Quat(*ee_rotations[ee_idx][0:4]))
            self.future_goal['positions'].append([Goal(v,0) for v in ee_positions[ee_idx][0:3]])
            self.future_goal['rotations'].append(Goal(Rotations.Quat(*ee_rotations[ee_idx][0:4]),0))

            # Add a marker for control
            name = self.info['ee_fixed_joints'][ee_idx]
            self.marker_lookup.append('interactive_marker_'+name)
            marker = self.create_marker(name,ee_positions[ee_idx][0:3],ee_rotations[ee_idx][0:4],self.info['fixed_frame'])
            self.ims.insert(marker,feedback_callback=self.feedback_cb)

        self.ims.applyChanges()

        self.current_goal['dc'] = self.info['starting_config']
        self.future_goal['dc'] = [Goal(v,0) for v in self.info['starting_config']]

        for objective in self.info['objectives']:
            self.current_goal['weights'].append(float(objective['weight']))
            self.future_goal['weights'].append(Goal(float(objective['weight']),0))

        self.objective_types = [objective['type'] for objective in self.info['objectives']]
        self.noise_objective_mask = [True if 'noise' in objective_name else False for objective in self.objective_types]
        rik_info = deepcopy(self.info)
        for idx,objective in rik_info['objectives']:
            if self.noise_objective_mask[idx]:
                rik_info['objectives'][idx] = 0.0
        rik_info['fixed_frame_noise_scale'] = 0.0
        self.seeds = [random.random()*1000 for j in self.info['joint_ordering']]

        self.lik_solver = LivelyIK.get_standard(yaml.dump(self.info))
        self.rik_solver = LivelyIK.get_standard(yaml.dump(rik_info))
        self.create_timer(0.05,self.step)

    def write_to_file(self):
        self.collecting = False
        with open(self.out_file,'w') as stream:
            stream.write(self.out_contents)
        self.out_contents = ''

    def start_collection(self,file):
        self.out_contents = ','.join(['condition']+self.info['joint_ordering'])
        self.out_file = file
        self.collecting = True

    def teardown(self):
        self.ims.clear()
        self.ims.applyChanges()
        if self.collecting:
            self.write_to_file()

    def feedback_cb(self,msg):
        #self.get_logger().info("EE CB: {0} {1}".format(idx, msg))
        idx = self.marker_lookup.index(msg.marker_name)
        position = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
        rotation = Rotations.Quat(msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z)
        self.set_position_goal(idx,position,.05)
        self.set_rotation_goal(idx,rotation,.05)

    def step(self):
        self.update_from_goals()
        standard_sol = LivelyIK.solve(self.rik_solver,
                                      self.current_goal['positions'],
                                      self.current_goal['rotations'],
                                      self.current_goal['dc'],
                                      self.time_as_seconds,
                                      self.current_goal['bias'],
                                      [0 if self.noise_objective_mask[i] else o for i,o in enumerate(self.current_goal['weights'])])
        lively_sol = LivelyIK.solve(self.lik_solver,
                             self.current_goal['positions'],
                             self.current_goal['rotations'],
                             self.current_goal['dc'],
                             self.time_as_seconds,
                             self.current_goal['bias'],
                             self.current_goal['weights'])
        self.current_goal['dc'] = standard_sol

        naive_sol = [v for v in standard_sol]
        for idx,joint in enumerate(self.info['joint_ordering']):
            if joint in ALLOWED_NAIVE_JOINTS[self.info['robot_name']]:
                naive_sol += LivelyIK.noise1D(self.time_as_seconds,self.seeds[idx],5)
                x, seed, freq
        # Publish
        js_msg = JointState(name=self.info['joint_ordering'],position=lively_sol)
        js_msg.header.stamp = self.time_as_msg
        self.js_pub.publish(js_msg)

        # Store data if collecting
        if self.collecting:
            self.out_contents += '\n'+','.join(['static']+[str(v) for v in standard_sol])
            self.out_contents += '\n'+','.join(['naive']+[str(v) for v in naive_sol])
            self.out_contents += '\n'+','.join(['lively']+[str(v) for v in lively_sol])

    def set_position_goal(self,idx,position,time=0):
        t = self.time_as_seconds
        self.future_goal['positions'][idx][0].update_goal(position.x,t)
        self.future_goal['positions'][idx][1].update_goal(position.y,t)
        self.future_goal['positions'][idx][2].update_goal(position.z,t)

    def set_rotation_goal(self,idx,rotation,time=0):
        t = self.time_as_seconds
        self.future_goal['rotations'][idx].update_goal(rotation,t+time)

    def set_dc_goal(self,idx,value,time=0):
        t = self.time_as_seconds
        self.future_goal['dc'][idx].update_goal(value,t+time)

    def set_bias_goal(self,idx,value,time=0):
        t = self.time_as_seconds
        self.future_goal['bias'][idx].update_goal(value,t+time)

    def set_weight_goal(self,idx,value,time=0):
        t = self.time_as_seconds
        self.future_goal['weights'][idx].update_goal(value,t+time)

    def update_from_goals(self):
        t = self.time_as_seconds
        # Positions
        for idx,goals in enumerate(self.future_goal['positions']):
            self.current_goal['positions'][idx] = [g.increment_towards(self.current_goal['positions'][i],t) for i,g in enumerate(goals)]
        # Rotations
        for idx,goal in enumerate(self.future_goal['rotations']):
            self.current_goal['rotations'][idx] = goal.increment_towards(self.current_goal['rotations'][idx],t)
        # DC
        for idx,goal in enumerate(self.future_goal['dc']):
            self.current_goal['dc'][idx] = goal.increment_towards(self.current_goal['dc'][idx],t)
        # Bias
        for idx,goal in enumerate(self.future_goal['bias']):
            self.current_goal['bias'][idx] = goal.increment_towards(self.current_goal['bias'][idx],t)
        # Objectives
        for idx,goal in enumerate(self.future_goal['weights']):
            self.current_goal['weights'][idx] = goal.increment_towards(self.current_goal['weights'][idx],t)


    @property
    def time_as_seconds(self):
        return self.node.get_clock().now().nanoseconds * 10**-9

    @property
    def time_as_msg(self):
        return self.node.get_clock().now().to_msg()

    @staticmethod
    def create_marker(name,ee_pos,ee_rot,frame_id):
        # create an interactive marker for our server
        marker = InteractiveMarker()
        marker.header.frame_id = frame_id
        marker.name = "interactive_marker_" + name
        marker.pose.position.x = float(ee_pos[0])
        marker.pose.position.y = float(ee_pos[1])
        marker.pose.position.z = float(ee_pos[2])
        marker.pose.orientation.w = float(ee_rot[0])
        marker.pose.orientation.x = float(ee_rot[1])
        marker.pose.orientation.y = float(ee_rot[2])
        marker.pose.orientation.z = float(ee_rot[3])
        marker.scale = 0.2

        # create a grey box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.07
        box_marker.scale.y = 0.07
        box_marker.scale.z = 0.07
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 0.6

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)

        marker.controls.append(box_control)

        # Add Translation Controls
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_1"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_2"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_3"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        marker.controls.append(control)

        # Add Rotation Controls
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "rotate_1"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_2"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "rotate_3"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        marker.controls.append(control)

        return marker
