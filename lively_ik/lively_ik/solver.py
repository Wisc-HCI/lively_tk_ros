import rclpy
from rclpy.node import Node

import json
from std_msgs.msg import String, Bool

from lively_ik.configuration.default import DEFAULT_CONFIG

from lively_ik_core import ObjectiveInput, LivelyIK, parse_config_data

from datetime import datetime

def ewma(target,last=None,a=0.6):
    if last:
        return target * a + (1.0-a) * last
    else:
        return target

class Time(object):
    def __init__(self,sec,nanosec):
        self.sec = sec
        self.nanosec = nanosec

class SolverNode(Node):
    def __init__(self):
        super(SolverNode,self).__init__('lively_ik_solver')
        self.config_data = DEFAULT_CONFIG
        self.config = parse_config_data(self.config_data)
        try:
            self.solver = LivelyIK(self.config)
        except:
            self.solver = None

        # Services and Topics
        self.config_sub = self.create_subscription(String, '/lively_ik/config_updates', self.handle_config_update, 10)
        self.weight_sub = self.create_subscription(String, '/lively_ik/weight_updates', self.handle_weight_update, 10)
        self.direct_sub = self.create_subscription(String, '/lively_ik/direct_updates', self.handle_direct_update, 10)
        self.enable_sub = self.create_subscription(Bool, '/lively_ik/enabled_updates', self.handle_enabled_update, 10)
        self.result_pub = self.create_publisher(String, '/lively_ik/result',10)

        self.enabled = False

        self.create_timer(.05,self.standard_loop)

        self.base_transform = [0,0,0]
        self.displayed_state = []
        self.target_weights = []
        self.current_weights = []
        self.target_directions = []
        self.current_directions = []

        self.get_logger().info('Initialized!')


    def handle_config_update(self,msg):
        self.get_logger().debug('Received request to update config')
        data = json.loads(msg.data)
        try:
            if self.config_data == data:
                pass
            else:
                self.get_logger().info('Changing config')
                self.config = parse_config_data(data)
                try:
                    self.solver = LivelyIK(self.config)
                except:
                    self.solver = None
                self.current_weights = self.config.modes[0].weights
                self.target_weights = self.current_weights
                self.current_directions = self.config.goals[0].values
                self.target_directions = self.current_directions
        except:
            self.get_logger().warning('Invalid config supplied')

    def handle_enabled_update(self,msg):
        self.get_logger().debug('Received request to update enabled setting')
        if msg.data != self.enabled:
            self.get_logger().info('Changing enabled to {0}'.format(msg.data))
            self.enabled = msg.data

    def handle_weight_update(self,msg):
        self.get_logger().debug('Received request to update weights')
        data = json.loads(msg.data)
        self.target_weights = data

    def handle_direct_update(self,msg):
        self.get_logger().debug('Received request to update directions')
        data = json.loads(msg.data)
        self.target_directions = data

    def update_current_weights(self):
        # If the weights were changed in size, just replace and don't interpolate.
        if len(self.current_weights) != len(self.target_weights):
            self.current_weights = self.target_weights
        else:
            for idx,target_weight in enumerate(self.target_weights):
                # Use ewma to interpolate to new values
                self.current_weights[idx] = ewma(target_weight,self.current_weights[idx],0.05)

    def update_current_directions(self):
        # If the weights were changed in size, just replace and don't interpolate.
        if len(self.current_directions) != len(self.target_directions):
            self.current_directions = self.target_directions
        else:
            for idx,target_direction in enumerate(self.target_directions):
                # Use ewma to interpolate to new values
                current_direction = self.current_directions[idx]
                for key in current_direction.keys():
                    if key not in target_direction.keys():
                        self.current_directions[idx] = target_direction
                    elif key == 'scalar':
                        self.current_directions[idx]['scalar'] = ewma(target_direction['scalar'],current_direction['scalar'],0.05)
                    else:
                        for dim in range(len(current_direction[key])):
                            self.current_directions[idx][key][dim] = ewma(target_direction[key][dim],current_direction[key][dim],0.05)

    def standard_loop(self):
        if self.enabled and self.solver != None:
            self.update_current_weights()
            self.update_current_directions()
            self.base_transform, self.displayed_state = self.solve_with_current_goals()
            data = {'base_transform':self.base_transform,'joint_states':self.displayed_state}
            self.result_pub.publish(String(data=json.dumps(data)))


    def solve_with_current_goals(self):
        inputs = []
        for idx,objective in enumerate(self.config.objectives):
            input_value = {}
            input_value.update({'weight':self.current_weights[idx]})
            input_value.update(self.current_directions[idx])
            inputs.append(ObjectiveInput(**input_value))
        return self.solver.solve(inputs,datetime.utcnow().timestamp())


def main():
    rclpy.init(args=None)
    node = SolverNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
