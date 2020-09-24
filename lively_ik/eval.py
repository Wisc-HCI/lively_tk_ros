import yaml
import lively_ik
import rclpy
from rclpy.node import Node
import os
from lively_ik import BASE
from std_msgs.msg import String, Float64
from wisc_actions.elements import Position, Orientation
from wisc_msgs.msg import GoalUpdate
from wisc_msgs.srv import UpdateGoals
import time
import json

class EvalNode(Node):
    def __init__(self):
        super(EvalNode,self).__init__('evaluator')
        self.manager_client = self.create_client(UpdateGoals,'goal_update')
        while not self.manager_client.wait_for_service(10.0):
            if rclpy.ok():
                self.get_logger().info("Waiting for service to be available")
            else:
                self.get_logger().error("Error while waiting for service")
                exit()
        self.setup_pub = self.create_publisher(String,'setup',10)
        self.configs = self.get_configs()

    @classmethod
    def get_configs(cls):
        configs = {}
        config_files = [f for f in os.listdir(BASE+'/config/info_files') if os.path.isfile(os.path.join(BASE+'/config/info_files', f))]
        for config_file in config_files:
            with open(BASE+'/config/info_files/'+config_file) as io:
                config_data = yaml.safe_load(io)
            if set(REQUIRED_CONFIG_PARAMETERS).issubset(set(config_data.keys())):
                configs[config_data['robot_name']] = config_data
        return configs

    @classmethod
    def create_nao_static_valence(cls,config,valence):
        # Valence is 'positive' or 'negative'
        container = RelaxedIKContainer(config)
        joints = config['starting_config']
        bias = [1.0,1.0,1.0]
        objective_weights = [obj['weight'] for obj in config['objectives']]
        if valence == 'positive':
            joints[1] = -0.15
            joints[2] = 1.6
            joints[3] = 0.45
            joints[7] = 1.6
            joints[8] = -0.45
            bias = [0.5,0.1,1.0]
            for idx,objective in enumerate(config['objectives']):
                if 'noise' in objective['type'] and objective['frequency'] == 3:

        elif valence == 'negative':
            joints[1] = 0.2
            joints[5] = -0.1
            joints[10] = 0.1
            bias = [1.0,0.1,0.1]


        ee_positions = container.robot.get_ee_positions(joints)
        ee_rotations = container.robot.get_ee_rotations(joints)

        commands = [{'updates':[],'delay':20}, # Prep
                    {'file':'nao_static_weights_{0}.csv'.format(valence),'valence':valence}, # Begin Recording
                    {'updates':[],'delay':30} # Task
                    {'record':''} # End Recording
                   ]
        for idx,position in enumerate(ee_positions):
            update = {'type':'position','value':Position(*position),'idx':idx,'time':5}
            for i in [0,2]:
                commands[i]['updates'].append(update)
        for idx,rotation in enumerate(ee_rotations):
            update = {'type':'rotation','value':Position(*rotation),'idx':idx,'time':5}
            for i in [0,2]:
                commands[i]['updates'].append(update)
        for idx,weight in enumerate(objective_weights):
            update = {'type':'weight','value':weight,'idx':idx,'time':5}
            for i in [0,2]:
                commands[i]['updates'].append(update)
        for idx,bias in enumerate(bias):
            update = {'type':'weight','value':weight,'idx':idx,'time':5}
            for i in [0,2]:
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
        bias = [0.5,0.1,1.0]
        objective_weights = [obj['weight'] for obj in config['objectives']]
        for objective in config['objectives']:
            if objective['type'] ==
        # if condition == 'static':
        #     bias = [0.0,0.0,0.0]
        #     for idx,objective in config['objectives']:
        #         if 'noise' in objective['type']:
        #             objective_weights[idx] = 0.0
        #         elif objective['type'] == 'dc':
        #             objective_weights[idx] = 40.0
        #         elif objective['type'] == 'position' or objective['type'] == 'rotation':
        #             objective_weights[idx] = 0.0
        # elif condition == 'naive':
        #     bias = [0.0,0.0,0.0]
        #     for idx,objective in config['objectives']:
        #         if objective['type'] == 'dc_noise':
        #             if config['joint_ordering'][objective['index']-1] in ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',
        #                                                                 'LAnklePitch', 'LAnkleRoll', 'RHipYawPitch', 'RHipRoll',
        #                                                                 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']:
        #                 objective_weights[idx] = 0.0
        #             else:
        #                 objective_weights[idx] = 40.0
        #         elif objective['type'] == 'dc' or 'position' in objective['type'] or 'rotation' in objective['type']:
        #             objective_weights[idx] = 0.0
        # elif condition == 'lively':
        #     # Defaults are fine
        #     pass

        ee_positions = container.robot.get_ee_positions(joints)
        ee_rotations = container.robot.get_ee_rotations(joints)

        commands = [{'updates':[],'delay':20}, # Prep
                    {'file':'nao_wave_task_{0}.csv'.format(condition),'condition':condition,'valence':'positive'}, # Begin Recording
                    {'updates':[],'delay':30} # Task
                    {'file':'','condition':condition} # End Recording
                   ]
        for idx,position in enumerate(ee_positions):
            update = {'type':'position','value':Position(*position),'idx':idx,'time':5}
            commands[0]['updates'].append(update)
        for idx,rotation in enumerate(ee_rotations):
            update = {'type':'rotation','value':Position(*rotation),'idx':idx,'time':5}
            commands[0]['updates'].append(update)
        for idx,weight in enumerate(objective_weights):
            update = {'type':'weight','value':weight,'idx':idx,'time':5}
            commands[0]['updates'].append(update)
        for idx,bias in enumerate(bias):
            update = {'type':'weight','value':weight,'idx':idx,'time':5}
            commands[0]['updates'].append(update)

        [RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw, RHand]
        # Action


        return commands

    def execute_nao(self):
        for valence in ['positive','negative']:
            for condition in ['static','naive','lively']:
                commands = self.create_nao_static_valence(self.configs['nao_v5'],condition,valence)
            self.execute_commands(commands)
        for condition in ['static','naive','lively']:
            commands = self.create_nao_wave_task(self.configs['nao_v5'],condition)
            self.execute_commands(commands)

    def execute_ur5(self):
        for valence in ['positive','negative']:
            for condition in ['static','naive','lively']:
                commands = self.create_ur5_static_valence(self.configs['ur5_robotiq_85'],condition,valence)
            self.execute_commands(commands)


    def execute_panda(self):
        for condition in ['static','naive','lively']:
            commands = self.create_panda_pickup_task(self.configs['panda'],condition)
            self.execute_commands(commands)

    def execute_commands(self,commands):
        for command in commands:
            update_msgs = []
            if 'file' in command:
                self.setup_pub.publish(String(json.dumps(command)))
            else:
                for update in command['updates']:
                    # Add GoalUpdate messages
                    if update['type'] == 'position':
                        gu = GoalUpdate(idx=update['idx'],position=update['value'].ros_vector3,type=0)
                    elif update['type'] == 'rotation':
                        gu = GoalUpdate(idx=update['idx'],rotation=update['value'].ros_quaternion,type=1)
                    elif update['type'] == 'dc':
                        gu = GoalUpdate(idx=update['idx'],dc=Float64(float(update['value'])),type=2)
                    elif update['type'] == 'bias':
                        gu = GoalUpdate(idx=update['idx'],bias=Float64(float(update['value'])),type=3)
                    elif update['type'] == 'weight':
                        gu = GoalUpdate(idx=update['idx'],weight=Float64(float(update['value'])),type=4)
                    updates.append(gu)
                update_request = UpdateGoals.Request(updates=updates)
                future = self.manager_client.call_async(update_request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if not response.results[0].success:
                    self.get_logger().warn('update failure')
                    break
                time.sleep(command['delay'])

def main():
    rclpy.init(args=None)

    node = EvalNode()

    rclpy.spin(node)
