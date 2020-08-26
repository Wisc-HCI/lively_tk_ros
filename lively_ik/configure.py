import rclpy
from rclpy.node import Node
from lively_ik.groove.relaxed_ik_container import RelaxedIKContainer

class ConfigureNode(Node):
    def __init__(self):
        super(ConfigureNode,self).__init__('configure')
        self.js_pub = self.create_publisher(JointState,'/joint_states',10)
        self.js_msg = JointState(name=JOINT_NAMES,position=JOINT_INITIAL)
        self.create_timer(0.05,self.send_js)

    def send_js(self):
        time = self.get_clock().now().nanoseconds * 10**-9
        self.js_msg.position[MOTION_JOINT] = .25*math.sin(time)+JOINT_INITIAL[MOTION_JOINT]
        self.js_pub.publish(self.js_msg)

def main():
    rclpy.init(args=None)

    node = ConfigureNode()

    rclpy.spin(node)
