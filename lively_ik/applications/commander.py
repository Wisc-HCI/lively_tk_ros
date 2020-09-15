import eventlet
eventlet.monkey_patch()
from threading import Thread
from lively_ik.applications.app import App
from lively_ik.utils.urdf_load import urdf_load_from_string
from lively_ik.spacetime.robot import Robot
from lively_ik import BASE, SRC
import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import inflection
import xml.etree.ElementTree as et
import yaml
from julia import LivelyIK
from flask_socketio import emit
import asyncio
import os

class Commander(App):

    def __init__(self, node):
        super(Commander,self).__init__(node,'Commander','commander')
        # self.js_pub = self.node.create_publisher(JointState,'/joint_states',10)
        #self.tf_pub = self.node.create_publisher(TFMessage,'tf_static',10)
        #self.js_timer = self.node.create_timer(0.05,self.publish_goals)
        #self.tf_timer = self.node.create_timer(1,self.publish_tf)
