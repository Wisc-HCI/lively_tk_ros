import yaml
import lively_ik
import rclpy
from rclpy.node import Node
import os
from lively_ik import BASE, INFO_PARAMS, get_configs
from lively_ik.groove.relaxed_ik_container import RelaxedIKContainer
from std_msgs.msg import String, Float64, Int16
from wisc_actions.elements import Position, Orientation
from wisc_msgs.msg import GoalUpdate, GoalUpdates
from wisc_msgs.srv import UpdateGoals
import time
import json

class CommanderNode(Node):
    def __init__(self):
        super(CommanderNode,self).__init__('evaluator')
        self.manager_pub = self.create_publisher(GoalUpdates,'/lively_apps/goal_update',10)
        self.setup_pub = self.create_publisher(String,'setup',10)
        self.configs = get_configs()

    @classmethod
    def joint_idx(cls,joint_name,config):
        return config['joint_ordering'].index(joint_name)

    @classmethod
    def objective_joint(cls,objective_idx,config):
        objective = config['objectives'][objective_idx]
        if objective['type'] in ['position','rotation','positional_noise','rotational_noise']:
            return config['ee_fixed_joints'][objective['index']-1]
        elif objective['type'] in ['dc','dc_noise']:
            return config['joint_ordering'][objective['index']-1]
        elif 'match' in objective['type']:
            return [config['joint_ordering'][objective['index_1']-1],config['joint_ordering'][objective['index_2']-1]]
        else:
            return None

    @classmethod
    def create_nao_static_valence(cls,config,valence):
        # Valence is 'positive' or 'negative'
        container = RelaxedIKContainer(config)
        joints = config['starting_config']
        bias = [1.0,1.0,1.0]
        objective_weights = [obj['weight'] for obj in config['objectives']]
        desired_frequency = 5
        if valence == 'positive':
            joints[cls.joint_idx('HeadPitch',config)] = -0.15
            joints[cls.joint_idx('LShoulderPitch',config)] = 1.6
            joints[cls.joint_idx('LShoulderRoll',config)] = 0.45
            joints[cls.joint_idx('RShoulderPitch',config)] = 1.6
            joints[cls.joint_idx('RShoulderPitch',config)] = -0.45
            bias = [0.5,0.1,1.0]
            desired_frequency = 3
        elif valence == 'negative':
            joints[cls.joint_idx('HeadPitch',config)] = 0.2
            joints[cls.joint_idx('LElbowRoll',config)] = -0.1
            joints[cls.joint_idx('RElbowRoll',config)] = 0.1
            bias = [1.0,0.1,0.1]
            desired_frequency = 10

        for idx,objective in enumerate(config['objectives']):
            if 'noise' in objective['type'] and objective['frequency'] == desired_frequency:
                if objective['type'] == 'positional_noise':
                    # Make the weight 20
                    objective_weights[idx] = 20
                elif objective['type'] == 'rotational_noise':
                    # Make the weight 5
                    objective_weights[idx] = 5
                elif objective['type'] == 'dc_noise' and cls.objective_joint(idx,config) in ['LHand','RHand']:
                    # Make the weight 20
                    objective_weights[idx] = 20
            elif 'noise' in objective['type']:
                # Make the weight 0
                objective_weights[idx] = 0

        ee_positions = container.robot.get_ee_positions(joints)
        ee_rotations = container.robot.get_ee_rotations(joints)

        commands = [{'config':'nao_v5','delay':120}, # Configuration
                    {'updates':[],'delay':20}, # Prep
                    {'file':'nao_static_weights_{0}.csv'.format(valence),'valence':valence}, # Begin Recording
                    {'updates':[],'delay':30}, # Task
                    {'file':'','valence':'neutral'} # End Recording
                   ]
        for idx,position in enumerate(ee_positions):
            update = {'type':'position','value':Position(*position),'idx':idx,'time':5}
            for i in [1,3]:
                commands[i]['updates'].append(update)
        for idx,rotation in enumerate(ee_rotations):
            update = {'type':'rotation','value':Orientation(*rotation),'idx':idx,'time':5}
            for i in [1,3]:
                commands[i]['updates'].append(update)
        for idx,weight in enumerate(objective_weights):
            update = {'type':'weight','value':weight,'idx':idx,'time':5}
            for i in [1,3]:
                commands[i]['updates'].append(update)
        for idx,bias in enumerate(bias):
            update = {'type':'bias','value':weight,'idx':idx,'time':5}
            for i in [1,3]:
                commands[i]['updates'].append(update)

        return commands

    @classmethod
    def create_ur5_static_valence(cls,config,condition,valence):
        # Condition is 'static','naive', or 'lively'
        # Valence is 'positive' or 'negative'
        container = RelaxedIKContainer(config)

        return []

    @classmethod
    def create_panda_pickup_task(cls,config,condition):
        # Condition is 'static','naive', or 'lively'
        container = RelaxedIKContainer(config)

        return []

    @classmethod
    def create_nao_wave_task(cls,config):
        # Condition is 'static','naive', or 'lively'
        container = RelaxedIKContainer(config)
        joints = config['starting_config']
        joints[cls.joint_idx('HeadPitch',config)] = -0.15
        joints[cls.joint_idx('LShoulderPitch',config)] = 1.6
        joints[cls.joint_idx('LShoulderRoll',config)] = 0.45
        joints[cls.joint_idx('RShoulderPitch',config)] = 1.6
        joints[cls.joint_idx('RShoulderPitch',config)] = -0.45
        bias = [0.5,0.1,1.0]
        objective_weights = [obj['weight'] for obj in config['objectives']]

        # TODO: Update objective weights

        ee_positions = container.robot.get_ee_positions(joints)
        ee_rotations = container.robot.get_ee_rotations(joints)

        commands = [{'config':'nao_v5'}, # Configuration
                    {'updates':[],'delay':20}, # Prep
                    {'file':'nao_wave_task.csv','valence':'positive'}, # Begin Recording
                   ]
        for idx,position in enumerate(ee_positions):
            update = {'type':'position','value':Position(*position),'idx':idx,'time':5}
            commands[1]['updates'].append(update)
        for idx,rotation in enumerate(ee_rotations):
            update = {'type':'rotation','value':Orientation(*rotation),'idx':idx,'time':5}
            commands[1]['updates'].append(update)
        for idx,weight in enumerate(objective_weights):
            update = {'type':'weight','value':weight,'idx':idx,'time':5}
            commands[1]['updates'].append(update)
        for idx,bias in enumerate(bias):
            update = {'type':'bias','value':weight,'idx':idx,'time':5}
            commands[1]['updates'].append(update)

        # Action

        wave = [{'RShoulderPitch':-1.1,'RWristYaw':-0.4,'RShoulderRoll':-0.47,'RElbowRoll':1.35,'RElbowYaw':0.0,'RHand':1.8},
                {'RShoulderPitch':-1.1,'RWristYaw':-0.4,'RShoulderRoll':-0.97,'RElbowRoll':0.35,'RElbowYaw':0.0,'RHand':1.8},
                {'RShoulderPitch':-1.1,'RWristYaw':-0.4,'RShoulderRoll':-0.47,'RElbowRoll':1.35,'RElbowYaw':0.0,'RHand':1.8},
                {'RShoulderPitch':-1.1,'RWristYaw':-0.4,'RShoulderRoll':-0.97,'RElbowRoll':0.35,'RElbowYaw':0.0,'RHand':1.8},
                {'RShoulderPitch':-1.1,'RWristYaw':-0.4,'RShoulderRoll':-0.47,'RElbowRoll':1.35,'RElbowYaw':0.0,'RHand':1.8}
        ]

        ## Weights Preparation
        command = {'updates':[],'delay':2}

        for idx,objective in enumerate(config['objectives']):
            ### Reduce weight on right arm position/orientation objectives
            if cls.objective_joint(idx,config) == 'RArm_effector_fixedjoint':
                update = {'type':'weight','value':0,'idx':idx,'time':1.5}
                command['updates'].append(update)
            ### Increase the weight on joint objectives for the right arm
            elif cls.objective_joint(idx,config) in ['RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll','RWristYaw']:
                if objective['type'] == 'dc_noise':
                    update = {'type':'weight','value':5,'idx':idx,'time':1.5}
                    command['updates'].append(update)
                elif objective['type'] == 'dc':
                    update = {'type':'weight','value':15,'idx':idx,'time':1.5}
                    command['updates'].append(update)

        commands.append(command)

        ## Wave proper
        for step in wave:
            command = {'updates':[],'delay':0.5}
            for joint,value in step.items():
                update = {'type':'dc','value':value,'idx':config['joint_ordering'].index(joint),'time':0.5}
                command['updates'].append(update)
            commands.append(command)

        ## Return to normal
        command = {'updates':[],'delay':1.5}
        for idx,position in enumerate(ee_positions):
            update = {'type':'position','value':Position(*position),'idx':idx,'time':1.5}
            command['updates'].append(update)
        for idx,rotation in enumerate(ee_rotations):
            update = {'type':'rotation','value':Orientation(*rotation),'idx':idx,'time':1.5}
            command['updates'].append(update)
        for idx,weight in enumerate(objective_weights):
            update = {'type':'weight','value':weight,'idx':idx,'time':1.5}
            command['updates'].append(update)

        commands.append(command)
        commands.append({'file':'','valence':'positive'})
        return commands

    def execute_nao(self):
        for valence in ['positive','negative']:
            commands = self.create_nao_static_valence(self.configs['nao_v5'],valence)
            self.execute_commands(commands)
        commands = self.create_nao_wave_task(self.configs['nao_v5'])
        self.execute_commands(commands)

    def execute_ur5(self):
        for valence in ['positive','negative']:
            commands = self.create_ur5_static_valence(self.configs['ur5_robotiq_85'],valence)
            self.execute_commands(commands)


    def execute_panda(self):
        commands = self.create_panda_pickup_task(self.configs['panda'])
        self.execute_commands(commands)

    def execute_commands(self,commands):
        for idx,command in enumerate(commands):
            update_msgs = []
            print(command.keys())
            if 'file' in command or 'config' in command:
                self.setup_pub.publish(String(data=json.dumps(command)))
            else:
                for update in command['updates']:
                    # Add GoalUpdate messages
                    if update['type'] == 'position':
                        gu = GoalUpdate(idx=Int16(data=update['idx']),position=update['value'].ros_vector3,type=Int16(data=0))
                    elif update['type'] == 'rotation':
                        gu = GoalUpdate(idx=Int16(data=update['idx']),rotation=update['value'].ros_quaternion,type=Int16(data=1))
                    elif update['type'] == 'dc':
                        gu = GoalUpdate(idx=Int16(data=update['idx']),dc=Float64(data=float(update['value'])),type=Int16(data=2))
                    elif update['type'] == 'bias':
                        gu = GoalUpdate(idx=Int16(data=update['idx']),bias=Float64(data=float(update['value'])),type=Int16(data=3))
                    elif update['type'] == 'weight':
                        gu = GoalUpdate(idx=Int16(data=update['idx']),weight=Float64(data=float(update['value'])),type=Int16(data=4))
                    update_msgs.append(gu)
                update_msg = GoalUpdates(updates=update_msgs)
                self.manager_pub.publish(update_msg)
            if 'delay' in command:
                self.get_logger().info('Waiting for {0}s'.format(command['delay']))
                time.sleep(command['delay'])
            self.get_logger().info('Command {0}/{1}'.format(idx+1,len(commands)))

def main():
    rclpy.init(args=None)

    node = CommanderNode()
    node.execute_nao()
    # node.execute_panda()
    # node.execute_ur5()
