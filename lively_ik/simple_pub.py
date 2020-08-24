import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint',
               'elbow_joint', 'wrist_1_joint',
               'wrist_2_joint', 'wrist_3_joint']
JOINT_INITIAL = [1.55, -1.77, 1.4, -1.19, -1.57, 0.0]
MOTION_JOINT = 2

class SimplePublisherNode(Node):
    def __init__(self):
        super(SimplePublisherNode,self).__init__('simple_js_publisher')
        self.js_pub = self.create_publisher(JointState,'/joint_states',10)
        self.js_msg = JointState(name=JOINT_NAMES,position=JOINT_INITIAL)
        self.create_timer(0.05,self.send_js)

    def send_js(self):
        time = self.get_clock().now().nanoseconds * 10**-9
        self.js_msg.position[MOTION_JOINT] = .25*math.sin(time)+JOINT_INITIAL[MOTION_JOINT]
        self.js_pub.publish(self.js_msg)

def main():
    rclpy.init(args=None)

    node = SimplePublisherNode()

    rclpy.spin(node)
