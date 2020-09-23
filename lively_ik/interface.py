import eventlet
eventlet.monkey_patch()
from threading import Thread


import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from wisc_msgs.msg import GoalUpdate
from wisc_msgs.srv import UpdateGoals

from flask import Flask, request, jsonify, make_response
from flask_socketio import SocketIO, emit, Namespace
from flask_cors import CORS

from lively_ik.applications.config_creator import ConfigCreator
from lively_ik.applications.commander import Commander

class InterfaceNamespace(Namespace):
    '''
    Namespace for handling SocketIO Events
    '''
    def __init__(self,node):
        super(InterfaceNamespace,self).__init__()
        self.node = node
        self.apps = {
            'config_creator':ConfigCreator(self.node),
            'commander':Commander(self.node)
        }
        self.active_app = None

    def on_connect(self):
        self.node.get_logger().info("Request: 'connect'")
        emit('get_app_list_response',{'apps':[app.info for key,app in self.apps.items()]})

    def on_get_app_list(self,data):
        self.node.get_logger().info("Request: 'get_app_list'")
        emit('get_app_list_response',{'apps':[app.info for key,app in self.apps.items()]})

    def on_set_active_app(self,data):
        self.node.get_logger().info("Request: 'select_app'")
        message = ''
        valid = False
        if data['app'] in self.apps.keys():
            self.active_app = data['app']
            valid = True
            message = 'App {0} selected'.format(self.active_app)
        else:
            message = 'App {0} not selected'.format(data['app'])
        emit('select_app_response',{'success':valid,'app':self.active_app,'message':message})

    def on_app_update(self,data):
        self.node.get_logger().info("Request: 'app_update'")
        if not self.active_app:
            self.node.get_logger().warn("Request: 'app_update failed: no active app'")
            emit('app_update_response',{'success':False})
        else:
            response = self.apps[self.active_app].update(data)
            emit('app_update_response',response)

    def on_app_process(self,data):
        self.node.get_logger().info("Request: 'app_process'")
        if not self.active_app:
            self.node.get_logger().warn("Request: 'app_process failed: no active app'")
            emit('app_process_response',{'success':False})
        else:
            self.apps[self.active_app].process(data)
            # process = self.apps[self.active_app].process(data)
            # for step in process:
            #     eventlet.greenthread.sleep()
            #     #self.node.get_logger().info(str(step))
            #     emit('app_process_response',step)


class InterfaceNode(Node):
    def __init__(self):
        super(InterfaceNode,self).__init__('lively_apps')
        self.app = Flask(__name__)
        self.cors = CORS(self.app)
        self.namespace = InterfaceNamespace(self)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        self.socketio.on_namespace(self.namespace)
        self.ros_thread = Thread(target=self.process,daemon=True)
        self.ros_thread.start()

        # Services and Topics
        self.robot_description_client = self.create_client(SetParameters, '/robot_state_publisher/set_parameters')
        self.js_pub = self.create_publisher(JointState,'/joint_states',10)
        self.tf_pub = self.create_publisher(TFMessage,'tf_static',10)
        self.goal_update_service = self.create_service(UpdateGoals,'goal_update',self.handle_goal_update)
        self.get_logger().info('Initialized!')

    def handle_goal_update(request,response):
        if self.namespace.active_app:
            return self.apps[self.active_app].handle_goal_update(request,response)
        else:
            response.success = False
            return response

    def update_robot_description(self,urdf):
        request = SetParameters.Request()
        request.parameters = [Parameter(name='robot_description',value=ParameterValue(type=4,string_value=urdf))]
        future = self.robot_description_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.results[0].successful

    def process(self):
        while rclpy.ok():
            rclpy.spin_once(self,timeout_sec=0.01)
            eventlet.greenthread.sleep()

def main():
    rclpy.init(args=None)

    node = InterfaceNode()

    node.socketio.run(node.app)

    node.destroy_node()
    rclpy.shutdown()
