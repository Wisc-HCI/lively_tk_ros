import yaml
import lively_ik
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from wisc_msgs.msg import JointAngles
from argparse import ArgumentParser
from rcl_interfaces.srv import GetParameters
import sys

class VisualizeNode(Node):
    def __init__(self):
        super(VisualizeNode,self).__init__('visualize')
        self.param_client = self.create_client(GetParameters, '/lively_ik_params/get_parameters')
        while not self.param_client.wait_for_service(1.0):
            if rclpy.ok():
                self.get_logger().info("Waiting for service to be available")
            else:
                exit()
        self.info = yaml.safe_load(self.get_lik_param('info'))
        self.js_pub = self.create_publisher(JointState,'/joint_states',5)
        self.ja_sub = self.create_subscription(JointAngles,'/relaxed_ik/joint_angles',self.ja_cb,10)

    def ja_cb(self,ja_msg):
        names = self.info['joint_ordering']
        positions = [angle.data for angle in ja_msg.angles]
        js_msg = JointState(header=ja_msg.header,name=names,position=positions)
        self.js_pub.publish(js_msg)

    def get_lik_param(self, param):
        request = GetParameters.Request()
        request.names = [param]
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.values[0].string_value



def main():
    rclpy.init(args=None)

    node = VisualizeNode()

    rclpy.spin(node)
