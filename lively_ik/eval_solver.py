from lively_ik.utils.urdf_load import urdf_load_from_string
from lively_ik.spacetime.robot import Robot
from lively_ik.utils.manager import Manager
from lively_ik import BASE, SRC, INFO_PARAMS, get_configs
from wisc_actions.elements import Position
from wisc_msgs.msg import GoalUpdate, GoalUpdates
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header, String
import xml.etree.ElementTree as et
import yaml
from julia import LivelyIK, Rotations
import os
import json

class SolverNode(Node):

    def __init__(self):
        super(SolverNode,self).__init__('solver')
        self.configs = get_configs()
        self.setup_sub = self.create_subscription(String,'setup',self.setup_cb,10)
        self.goal_sub = self.create_subscription(GoalUpdates,'/lively_apps/goal_update',self.goal_update_cb,10)
        self.manager = None

    def set_robot(self,config):
        if self.manager and self.manager.info['robot_name'] == config:
            # The manager exists and matches, so just return
            self.get_logger().info('Robot config already matches. Ignoring.')
            return
        if self.manager and self.manager.info['robot_name'] != config:
            # If the manager exists and represents a different robot, tear it down
            self.get_logger().info('Tearing down previous configuration.')
            self.manager.teardown()
            self.manager = None
        if self.manager == None:
            # Create a new manager for the new robot
            self.get_logger().info('Creating new configuration.')
            self.manager = Manager(self,self.configs[config])

    def set_output(self,out_file):
        if out_file == '' and self.manager and self.manager.collecting:
            self.manager.write_to_file()
        elif out_file != '' and self.manager and not self.manager.collecting:
            self.manager.start_collection(SRC+'/eval/'+out_file)
        elif out_file == '':
            self.get_logger().info('output file is empty')
        elif out_file != '':
            self.get_logger().info('output file is non-empty')

    def setup_cb(self,msg):
        setup_info = json.loads(msg.data)
        executed = False
        if 'file' in setup_info:
            self.get_logger().info('File -> {0}'.format(setup_info['file']))
            self.set_output(setup_info['file'])
            executed = True
        if 'valence' in setup_info and self.manager:
            self.get_logger().info('Setting valence to {0}'.format(setup_info['valence']))
            self.manager.valence = setup_info['valence']
            executed = True
        if 'config' in setup_info:
            self.get_logger().info('Setting Robot to {0}'.format(setup_info['config']))
            self.set_robot(setup_info['config'])
            self.get_logger().info('Finished setting up robot {0}'.format(setup_info['config']))
            executed = True
        if not executed:
            self.get_logger().warn('Unknown command: {0}'.format(setup_info))


    def goal_update_cb(self,msg):
        self.get_logger().info("Received update request")
        if self.manager:
            for goal_update in msg.updates:
                idx = goal_update.idx.data
                t = goal_update.time.data
                if goal_update.type.data == 0:
                    position = Position.from_ros_point(goal_update.position)
                    self.manager.set_position_goal(idx,position,t)
                elif goal_update.type.data == 1:
                    # Update rotation goal
                    rotation = Rotations.Quat(goal_update.rotation.w,goal_update.rotation.x,goal_update.rotation.y,goal_update.rotation.z)
                    self.manager.set_rotation_goal(idx,rotation,t)
                elif goal_update.type.data == 2:
                    # Update dc goal
                    self.manager.set_dc_goal(idx,goal_update.dc.data,t)
                elif goal_update.type.data == 3:
                    # Update bias goal
                    self.manager.set_bias_goal(idx,goal_update.bias.data,t)
                elif goal_update.type.data == 4:
                    # Update weight goal
                    self.manager.set_weight_goal(idx,goal_update.weight.data,t)

def main():
    rclpy.init(args=None)

    node = SolverNode()
    node.get_logger().info('Initialized!')
    rclpy.spin(node)
