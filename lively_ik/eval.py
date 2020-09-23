import yaml
import lively_ik
import rclpy
from rclpy.node import Node
import os
from lively_ik import BASE

def get_configs(self):
    configs = {}
    config_files = [f for f in os.listdir(BASE+'/config/info_files') if os.path.isfile(os.path.join(BASE+'/config/info_files', f))]
    for config_file in config_files:
        with open(BASE+'/config/info_files/'+config_file) as io:
            config_data = yaml.safe_load(io)
        if set(REQUIRED_CONFIG_PARAMETERS).issubset(set(config_data.keys())):
            configs[config_data['robot_name']] = config_data
    return configs


nao_wave_test = [
[{'position':{'x':0,'y':0,'z':0},
  'idx':0,
  'time':0},
 {'position':{'x':0,'y':0,'z':0},
  'idx':1,
  'time':0},
 {'position':{'x':0,'y':0,'z':0},
  'idx':1,
  'time':0},
 {'position':{'x':0,'y':0,'z':0},
  'idx':1,
  'time':0},
 {'position':{'x':0,'y':0,'z':0},
  'idx':1,
  'time':0}]
]

class EvalNode(Node):
    def __init__(self):
        super(EvalNode,self).__init__('evaluator')
        self.manager_client = self.create_client(GoalUpdate,'goal_update')
        while not self.manager_client.wait_for_service(10.0):
            if rclpy.ok():
                self.get_logger().info("Waiting for service to be available")
            else:
                self.get_logger().error("Error while waiting for service")
                exit()



def main():
    rclpy.init(args=None)

    node = EvalNode()

    rclpy.spin(node)
