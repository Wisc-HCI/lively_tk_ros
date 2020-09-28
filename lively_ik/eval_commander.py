import yaml
import lively_ik
import rclpy
from rclpy.node import Node
import os
from lively_ik import BASE, INFO_PARAMS, get_configs
from julia import LivelyIK
from lively_ik.groove.relaxed_ik_container import RelaxedIKContainer
from std_msgs.msg import String, Float64, Int16
from wisc_actions.elements import Pose, Position, Orientation, ModeTrajectory, PoseTrajectory
from wisc_msgs.msg import GoalUpdate, GoalUpdates, LivelyGoals, EvalResult
from wisc_msgs.srv import UpdateGoals
import time
import json

class Test(object):
    def __init__(self,name,config,valence):
        self.name = name
        self.config = config
        self.container = RelaxedIKContainer(config)
        self.valence = valence
        self.bias = [0.5,0.5,0.5]
        self.frequency = 5
        if valence == 'positive':
            self.bias = [0.5,0.1,1.0]
            self.frequency = 3
        elif valence == 'negative':
            self.bias = [1.0,0.1,0.1]
            self.frequency = 10
        self.initial_joints = config['starting_config']
        self.objectives = config['objectives']
        self.initial_weights = [obj['weight'] for obj in self.objectives]
        self.objective_types = [obj['type'] for obj in self.objectives]
        self.contents = ','.join(['condition']+self.config['joint_ordering'])

    @property
    def initial_positions(self):
        return self.container.robot.get_ee_positions(self.initial_joints)

    @property
    def inital_rotations(self):
        return self.container.robot.get_ee_rotations(self.initial_joints)

    @property
    def update_description(self):
        return []

    def __len__(self):
        return self.duration

    def compile(self):
        last_update = 0
        update_description = self.update_description
        compiled = {
            'pose':[],
            'weight':[],
            'dc':[],
            'bias':[ModeTrajectory([{'time':0,'mode':self.bias[0]}]),
                    ModeTrajectory([{'time':0,'mode':self.bias[1]}]),
                    ModeTrajectory([{'time':0,'mode':self.bias[2]}])]
        }
        raw_pose_trajectories = []
        raw_weight_trajectories = []
        raw_dc_trajectories = []
        for pose in self.poses_from_joints(self.initial_joints):
            raw_pose_trajectories.append([{'time':0,'pose':pose}])
        for weight in self.initial_weights:
            raw_weight_trajectories.append([{'time':0,'mode':weight}])
        for dc in self.initial_joints:
            raw_dc_trajectories.append([{'time':0,'mode':dc}])
        for update in update_description:
            if update['time'] > last_update:
                last_update = update['time']
            if update['type'] == 'pose':
                raw_pose_trajectories[update['idx']].append({'time':update['time'],'pose':update['value']})
            elif update['type'] == 'weight':
                raw_weight_trajectories[update['idx']].append({'time':update['time'],'mode':update['value']})
            elif update['type'] == 'dc':
                raw_dc_trajectories[update['idx']].append({'time':update['time'],'mode':update['value']})
        for pose_trajectory in raw_pose_trajectories:
            compiled['pose'].append(PoseTrajectory(pose_trajectory))
        for weight_trajectory in raw_weight_trajectories:
            compiled['weight'].append(ModeTrajectory(weight_trajectory))
        for dc_trajectory in raw_dc_trajectories:
            compiled['dc'].append(ModeTrajectory(dc_trajectory))
        self.compiled = compiled
        self.duration = last_update

    def joint_idx(self,joint_name):
        return self.config['joint_ordering'].index(joint_name)

    def objective_joint(self,objective_idx):
        objective = self.config['objectives'][objective_idx]
        if objective['type'] in ['position','rotation','positional_noise','rotational_noise']:
            return self.config['ee_fixed_joints'][objective['index']-1]
        elif objective['type'] in ['dc','dc_noise']:
            return self.config['joint_ordering'][objective['index']-1]
        elif 'match' in objective['type']:
            return [self.config['joint_ordering'][objective['index_1']-1],self.config['joint_ordering'][objective['index_2']-1]]
        else:
            return None

    def positions_from_joints(self,joints):
        return [Position(pos[0],pos[1],pos[2]) for pos in self.container.robot.get_ee_positions(joints)]

    def rotations_from_joints(self,joints):
        return [Orientation(ori[0],ori[1],ori[2],ori[3]) for ori in self.container.robot.get_ee_rotations(joints)]

    def poses_from_joints(self,joints):
        positions = self.positions_from_joints(joints)
        rotations = self.rotations_from_joints(joints)
        poses = []
        for i in range(len(positions)):
            poses.append(Pose(positions[i],rotations[i]))
        return poses

    @property
    def initial(self):
        return self[0]

    def __getitem__(self,time):
        update = {
            'pose':[],
            'dc':[],
            'bias':[],
            'weight':[]
        }
        for pose_trajectory in self.compiled['pose']:
            update['pose'].append(pose_trajectory[time])
        for mode_trajectory in self.compiled['dc']:
            update['dc'].append(mode_trajectory[time])
        for mode_trajectory in self.compiled['weight']:
            update['weight'].append(mode_trajectory[time])
        for mode_trajectory in self.compiled['bias']:
            update['bias'].append(mode_trajectory[time])
        return update

    def execute(self,node):
        self.active = True
        self.node = node
        # Set the callback for listening
        self.node.get_logger().info('Setting Listener for {0}'.format(self.name))
        self.node.results_cb = self.results_cb

        # Initialize for 60 seconds
        self.node.get_logger().info('Initializing {0}'.format(self.name))
        start_time = self.node.time_as_seconds
        elapsed_time = 0
        while elapsed_time <= 60:
            self.node.goal_pub.publish(self.node.create_update(self.initial,metadata=''))
            elapsed_time = self.node.time_as_seconds - start_time

        # Execute for the length of the task
        self.node.get_logger().info('Executing {0}'.format(self.name))
        start_time = self.node.time_as_seconds
        elapsed_time = 0
        while elapsed_time <= self.duration:
            self.node.goal_pub.publish(self.node.create_update(self[elapsed_time],metadata=self.name))
            elapsed_time = self.node.time_as_seconds - start_time
        self.node.get_logger().info('Task Complete {0}'.format(self.name))

        while self.active:
            rclpy.spin_once(node)

    def results_cb(self,msg):
        metadata = msg.metadata.data
        current_time = self.node.time_as_seconds
        if metadata == '' and self.active:
            self.active = False
            with open(SRC+'/eval/'+self.name+'.csv','w') as stream:
                stream.write(self.contents)
            return
        elif metadata == '':
            return
        r_joints = [d.data for d in msg.relaxed_joints]
        l_joints = [d.data for d in msg.lively_joints]

        n_joints = [v for v in r_joints]
        for idx,joint in enumerate(self.config['joint_ordering']):
            if joint in ALLOWED_NAIVE_JOINTS[self.config['robot_name']]:
                n_joints[idx] += LivelyIK.noise1D(current_time,self.seeds[idx],self.valence_lookup[self.valence]) * JOINT_SCALES[self.config['robot_name']][joint]
        n_joints = [clamp(v,self.config['joint_limits'][i][0],self.config['joint_limits'][i][1]) for i,v in enumerate(r_joints)]

        self.contents += '\n'+','.join(['static']+[str(v) for v in r_joints])
        self.contents += '\n'+','.join(['naive']+[str(v) for v in n_joints])
        self.contents += '\n'+','.join(['lively']+[str(v) for v in l_joints])


class NaoStaticValence(Test):
    def __init__(self,valence):
        config = get_configs()['nao_v5']
        super(NaoStaticValence,self).__init__('nao_static_weights_{0}'.format(valence),config,valence)
        if valence == 'positive':
            self.initial_joints[self.joint_idx('HeadPitch')] = -0.15
            self.initial_joints[self.joint_idx('LShoulderPitch')] = 1.6
            self.initial_joints[self.joint_idx('LShoulderRoll')] = 0.45
            self.initial_joints[self.joint_idx('RShoulderPitch')] = 1.6
            self.initial_joints[self.joint_idx('RShoulderPitch')] = -0.45
        elif valence == 'negative':
            self.initial_joints[self.joint_idx('HeadPitch')] = 0.2
            self.initial_joints[self.joint_idx('LElbowRoll')] = -0.1
            self.initial_joints[self.joint_idx('RElbowRoll')] = 0.1
        for idx,objective in enumerate(self.config['objectives']):
            if 'noise' in objective['type'] and objective['frequency'] == self.frequency:
                if objective['type'] == 'positional_noise':
                    # Make the weight 20
                    self.initial_weights[idx] = 20
                elif objective['type'] == 'rotational_noise':
                    # Make the weight 5
                    self.initial_weights[idx] = 5
                elif objective['type'] == 'dc_noise' and self.objective_joint(idx) in ['LHand','RHand']:
                    # Make the weight 20
                    self.initial_weights[idx] = 20
            elif 'noise' in objective['type']:
                # Make the weight 0
                self.initial_weights[idx] = 0
        self.compile()

    @property
    def update_description(self):
        commands = []
        for idx,pose in enumerate(self.poses_from_joints(self.initial_joints)):
            update = {'type':'pose','value':pose,'idx':idx,'time':30}
            commands.append(update)
        for idx,weight in enumerate(self.initial_weights):
            update = {'type':'weight','value':weight,'idx':idx,'time':30}
            commands.append(update)
        for idx,bias in enumerate(self.bias):
            update = {'type':'bias','value':bias,'idx':idx,'time':30}
            commands.append(update)
        for idx,dc in enumerate(self.initial_joints):
            update = {'type':'dc','value':dc,'idx':idx,'time':30}
            commands.append(update)
        return commands

class NaoWaveAction(Test):
    def __init__(self):
        config = get_configs()['nao_v5']
        super(NaoWaveAction,self).__init__('nao_wave_task',config,'positive')
        self.initial_joints[self.joint_idx('HeadPitch')] = -0.15
        self.initial_joints[self.joint_idx('LShoulderPitch')] = 1.6
        self.initial_joints[self.joint_idx('LShoulderRoll')] = 0.45
        self.initial_joints[self.joint_idx('RShoulderPitch')] = 1.6
        self.initial_joints[self.joint_idx('RShoulderPitch')] = -0.45
        for idx,objective in enumerate(self.config['objectives']):
            if 'noise' in objective['type'] and objective['frequency'] == self.frequency:
                if objective['type'] == 'positional_noise':
                    # Make the weight 20
                    self.initial_weights[idx] = 20
                elif objective['type'] == 'rotational_noise':
                    # Make the weight 5
                    self.initial_weights[idx] = 5
                elif objective['type'] == 'dc_noise' and self.objective_joint(idx) in ['LHand','RHand']:
                    # Make the weight 20
                    self.initial_weights[idx] = 20
            elif 'noise' in objective['type']:
                # Make the weight 0
                self.initial_weights[idx] = 0
        self.compile()

    @property
    def update_description(self):
        commands = []

        for idx,objective in enumerate(self.config['objectives']):
            ### Reduce weight on right arm position/orientation objectives
            if self.objective_joint(idx) == 'RArm_effector_fixedjoint':
                update = {'type':'weight','value':0,'idx':idx,'time':5}
                commands.append(update)
            ### Increase the weight on joint objectives for the right arm
            elif self.objective_joint(idx) in ['RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll','RWristYaw']:
                if objective['type'] == 'dc_noise' and objective['frequency'] == self.frequency:
                    update = {'type':'weight','value':5,'idx':idx,'time':5}
                    commands.append(update)
                elif objective['type'] == 'dc':
                    update = {'type':'weight','value':15,'idx':idx,'time':5}
                    commands.append(update)
                elif objective['type'] == 'dc_noise' and objective['frequency'] != self.frequency:
                    update = {'type':'weight','value':0,'idx':idx,'time':5}

        wave = [{'RShoulderPitch':-2.3,'RWristYaw':-0.4,'RShoulderRoll':0,'RElbowRoll':1.35,'RElbowYaw':0.0},
                {'RShoulderPitch':-2.3,'RWristYaw':-0.4,'RShoulderRoll':0.1,'RElbowRoll':0.35,'RElbowYaw':0.0},
                {'RShoulderPitch':-2.3,'RWristYaw':-0.4,'RShoulderRoll':-0.6,'RElbowRoll':1.35,'RElbowYaw':0.0},
                {'RShoulderPitch':-2.3,'RWristYaw':-0.4,'RShoulderRoll':0.1,'RElbowRoll':0.35,'RElbowYaw':0.0},
                {'RShoulderPitch':-2.3,'RWristYaw':-0.4,'RShoulderRoll':-0.6,'RElbowRoll':1.35,'RElbowYaw':0.0}
        ]


        joints = [v for v in self.initial_joints]
        time = 6
        for step in wave:
            for joint_name, dc in step.items():
                idx = self.config['joint_ordering'].index(joint_name)
                commands.append({'type':'dc','value':dc,'idx':idx,'time':time})
            time += 0.5

        for time in [15,30]:
            for idx,pose in enumerate(self.poses_from_joints(self.initial_joints)):
                update = {'type':'pose','value':pose,'idx':idx,'time':time}
                commands.append(update)
            for idx,weight in enumerate(self.initial_weights):
                update = {'type':'weight','value':weight,'idx':idx,'time':time}
                commands.append(update)
            for idx,bias in enumerate(self.bias):
                update = {'type':'bias','value':bias,'idx':idx,'time':time}
                commands.append(update)
            for idx,dc in enumerate(self.initial_joints):
                update = {'type':'dc','value':dc,'idx':idx,'time':time}
                commands.append(update)

        return commands


class CommanderNode(Node):
    def __init__(self):
        super(CommanderNode,self).__init__('evaluator')
        #self.manager_pub = self.create_publisher(GoalUpdates,'/lively_apps/goal_update',10)
        self.results_cb = lambda msg: None
        self.goal_pub = self.create_publisher(LivelyGoals,'/robot_goals',10)
        self.results_pub = self.create_subscription(EvalResult,'/eval_results',self.results_cb,10)
        self.active_task = None

    def create_update(self,update,metadata=''):
        metadata_msg = String(data=metadata)
        pose_msg     = [pose.ros_pose for pose in update['pose']]
        dc_msg       = [Float64(data=float(dc)) for dc in update['dc']]
        weight_msg   = [Float64(data=float(weight)) for weight in update['weight']]
        bias_msg     = Position(*update['bias']).ros_point

        msg = LivelyGoals(metadata=metadata_msg,
                          ee_poses=pose_msg,
                          dc_values=dc_msg,
                          objective_weights=weight_msg,
                          bias=bias_msg)

        return msg

    @property
    def time_as_seconds(self):
        return self.get_clock().now().nanoseconds * 10**-9

    @property
    def time_as_msg(self):
        return self.get_clock().now().to_msg()

def clamp(x, lo, hi):
    if lo <= x <= hi:
        return x
    elif x < lo:
        return lo
    elif x > hi:
        return hi

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

def main():
    rclpy.init(args=None)

    node = CommanderNode()
    NaoStaticValence('positive').execute(node)
    NaoStaticValence('negative').execute(node)
    NaoWaveAction().execute(node)
    # node.execute_panda()
    # node.execute_ur5()
