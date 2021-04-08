import yaml
import lively_ik
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from wisc_msgs.msg import JointAngles
from argparse import ArgumentParser
import sys
from rclpy.parameter import Parameter

class ParameterNode(Node):
    def __init__(self):
        super(ParameterNode,self).__init__('global_params',allow_undeclared_parameters=True)
        self.declare_parameter('info')
        self.declare_parameter('output_topic')
        self.declare_parameter('bias')

def main():
    rclpy.init(args=None)

    node = ParameterNode()

    rclpy.spin(node)
