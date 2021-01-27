import rclpy
from rclpy.node import Node

import json

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

import tf2_ros

from lively_ik.configuration.config_manager import ConfigManager

from datetime import datetime

class InterfaceNode(Node):
    def __init__(self):
        super(InterfaceNode,self).__init__('lively_ik')
        self.config_manager = ConfigManager()

        # Services and Topics
        self.robot_description_client = self.create_client(SetParameters, '/robot_state_publisher/set_parameters')
        self.js_pub = self.create_publisher(JointState,'/joint_states',10)
        self.tf_pub = self.create_publisher(TFMessage,'tf_static',10)

        self.gui_sub = self.create_subscription(String, '/lively_ik/gui_updates', self.handle_gui_update, 10)
        self.gui_pub = self.create_publisher(String, '/lively_ik/server_updates', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info('Initialized!')
        self.create_timer(.1,self.standard_loop)

        self.base_transform = [0,0,0]
        self.displayed_state = []
        self.manual_display = False

    def handle_gui_update(self,msg):
        self.get_logger().info('Interface: Received update request')
        data = json.loads(msg.data)
        if data['directive'] == 'update':
            if 'config' in data.keys():
                self.config_manager.load(data['config'])
            if 'meta' in data.keys():
                if 'displayed_state' in data['meta'].keys():
                    self.config_manager.displayed_state = data['meta']['displayed_state']
                if 'control' in data['meta'].keys():
                    self.config_manager.control = data['meta']['control']
        elif data['directive'] == 'train':
            self.get_logger().info('Should train!')
        self.pub_to_gui({'directive':'update','config':self.config_manager.data,'meta':self.config_manager.meta})
        if 'config' in data and 'urdf' in data['config'] and self.config_manager.valid_urdf:
            self.get_logger().info('Updating robot description!')
            self.update_robot_description(self.config_manager.urdf)
            self.get_logger().info('Updated robot description!')

    def standard_loop(self):
        if self.config_manager.valid_solver and self.config_manager.control == 'solve':
            # self.get_logger().info('solving')
            self.base_transform, self.displayed_state = self.solve_with_default_goals()
        elif self.config_manager.valid_urdf:
            # self.get_logger().info('{0} {1}'.format(self.config_manager.valid_solver,self.config_manager.control))
            self.base_transform, self.displayed_state = ([0,0,0], self.config_manager.displayed_state)

        # Send the transform for the base
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = self.config_manager.fixed_frame
        t.transform.translation.x = float(self.base_transform[0])
        t.transform.translation.y = float(self.base_transform[1])
        t.transform.translation.z = float(self.base_transform[2])
        self.tf_broadcaster.sendTransform(t)

        # Send the joint states
        js = JointState(name=self.config_manager.joint_ordering,position=self.displayed_state)
        js.header.stamp = self.get_clock().now().to_msg()
        self.js_pub.publish(js)

    def solve_with_default_goals(self):
        return self.config_manager.solver.solve(self.config_manager.config.default_goals,datetime.utcnow().timestamp())

    def pub_to_gui(self,data):
        data_msg = String(data=json.dumps(data))
        self.gui_pub.publish(data_msg)

    def update_robot_description(self,urdf):
        request = SetParameters.Request()
        request.parameters = [Parameter(name='robot_description',value=ParameterValue(type=4,string_value=urdf))]
        future = self.robot_description_client.call_async(request)
        #rclpy.spin_until_future_complete(self, future)

def main():
    rclpy.init(args=None)
    node = InterfaceNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
