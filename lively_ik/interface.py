import eventlet
eventlet.monkey_patch()
from threading import Thread


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from flask import Flask, request, jsonify, make_response
from flask_socketio import SocketIO, emit, Namespace
from flask_cors import CORS

from lively_ik.applications.config_creator import ConfigCreator

class InterfaceNamespace(Namespace):
    '''
    Namespace for handling SocketIO Events
    '''
    def __init__(self,node):
        super(InterfaceNamespace,self).__init__()
        self.node = node
        self.apps = {
            'config_creator':ConfigCreator(self.node)
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
            # self.apps[self.active_app].process(data)
            process = self.apps[self.active_app].process(data)
            for step in process:
                eventlet.greenthread.sleep()
                print(step)
                emit('app_process_response',step)


class InterfaceNode(Node):
    def __init__(self):
        super(InterfaceNode,self).__init__('lively_apps')
        self.app = Flask(__name__)
        self.cors = CORS(self.app)
        self.namespace = InterfaceNamespace(self)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        self.socketio.on_namespace(self.namespace)
        #self.process_timer = self.create_timer(.1,self.namespace.tick)
        self.ros_thread = Thread(target=self.process,daemon=True)
        self.ros_thread.start()
        # self.publish_initial()
        self.get_logger().info('Initialized!')

    def process(self):
        while rclpy.ok():
            rclpy.spin_once(self,timeout_sec=0.01)
            eventlet.greenthread.sleep()

    # def publish_initial(self):
    #     js_pub = self.create_publisher(JointState,'/joint_states',10)
    #     js_msg = JointState(name=['default_link'],position=[0])
    #     js_msg.header.stamp = self.node.get_clock().now().to_msg()
    #     js_pub.publish(js_msg)

def main():
    rclpy.init(args=None)

    node = InterfaceNode()

    node.socketio.run(node.app)

    node.destroy_node()
    rclpy.shutdown()
