import eventlet
eventlet.monkey_patch()
from threading import Thread
from lively_ik.applications.app import App
from lively_ik.utils.urdf_load import urdf_load_from_string
from lively_ik.spacetime.robot import Robot
from lively_ik.utils.manager import Manager
from lively_ik import BASE, SRC, INFO_PARAMS, get_configs
from wisc_actions.elements import Position
from wisc_msgs.msg import GoalUpdate
from wisc_msgs.srv import UpdateGoals
import rclpy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header, String
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import xml.etree.ElementTree as et
import yaml
from julia import LivelyIK, Rotations
import os
import json

class Commander(App):

    def __init__(self, node):
        super(Commander,self).__init__(node,'Commander','commander')
        self.configs = get_configs()
        self.tf_timer = self.node.create_timer(1,self.publish_tf)
        self.setup_sub = self.node.create_subscription(String,'setup',self.setup_cb,10)
        self.manager = None

    def set_robot(self,config):
        if self.manager:
            self.manager.teardown()
        self.node.update_robot_description(self.configs[config]['urdf'])
        self.manager = Manager(self.node,self.configs[config])

    def set_output(self,out_file,valence):
        if out_file == '' and self.manager.collecting:
            self.manager.write_to_file()
        elif out_file != '':
            self.manager.valence = valence
            self.manager.start_collection(BASE+'/eval/'+out_file)

    def setup_cb(self,msg):
        setup_info = json.loads(msg.data)
        if 'file' in setup_info and 'valence' in setup_info:
            self.log('File -> {0}'.format(setup_info['file']))
            self.set_output(setup_info['file'],setup_info['valence'])
        elif 'config' in setup_info:
            self.log('Setting Robot to {0}'.format(setup_info['config']))
            self.set_robot(setup_info['config'])
        else:
            self.warn('Unknown command: {0}'.format(setup_info))

    def publish_tf(self):
        if self.manager:
            tf1 = TransformStamped(header=Header(stamp=self.node.get_clock().now().to_msg(),
                                                 frame_id='1'),
                                   child_frame_id='world',
                                   transform=Transform(translation=Vector3(x=0.0,y=0.0,z=0.0),
                                                       rotation=Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)))
            tf2 = TransformStamped(header=Header(stamp=self.node.get_clock().now().to_msg(),
                                                 frame_id='world'),
                                   child_frame_id=self.manager.info['fixed_frame'],
                                   transform=Transform(translation=Vector3(x=0.0,y=0.0,z=0.0),
                                                       rotation=Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)))

            tf_msg = TFMessage(transforms=[tf1,tf2])
            self.node.tf_pub.publish(tf_msg)

    def update(self,data):
        if data['action'] == 'update_position_goal' and self.manager:
            position_goal = Position(data['value']['x'],data['value']['y'],data['value']['z'])
            return self.manager.set_position_goal(data['idx'],position_goal,data['time'])
        elif data['action'] == 'update_rotation_goal' and self.manager:
            rotation_goal = Rotations.Quat(w=data['value']['w'],x=data['value']['x'],y=data['value']['y'],z=data['value']['z'])
            return self.manager.set_rotation_goal(data['idx'],rotation_goal,data['time'])
        elif data['action'] == 'update_dc_goal' and self.manager:
            return self.manager.set_dc_goal(data['idx'],data['value'],data['time'])
        elif data['action'] == 'update_bias_goal' and self.manager:
            return self.manager.set_bias_goal(data['idx'],data['value'],data['time'])
        elif data['action'] == 'update_weight_goal' and self.manager:
            return self.manager.set_weight_goal(data['idx'],data['value'],data['time'])
        elif data['action'] == 'fetch':
            return {'success':True,'action':'fetch','config':self.json_config,'app':self.json_app}

    def handle_goal_update(self,request,response):
        self.log("Received update request")
        if self.manager:
            response.success = True
            for goal_update in request.updates:
                idx = goal_update.idx.data
                t = goal_update.time.data
                if goal_update.type.data == 0:
                    position = Position.from_ros_point(goal_update.position)
                    self.manager.set_position_goal(idx,position,t)
                elif goal_update.type.data == 1:
                    # Update rotation goal
                    rotation = Rotations.Quat(w=goal_update.rotation.w,x=goal_update.rotation.x,y=goal_update.rotation.y,z=goal_update.rotation.z)
                    self.manager.set_position_goal(idx,position,t)
                elif goal_update.type.data == 2:
                    # Update dc goal
                    self.manager.set_dc_goal(idx,goal_update.dc.data,t)
                elif goal_update.type.data == 3:
                    # Update bias goal
                    self.manager.set_bias_goal(idx,goal_update.bias.data,t)
                elif goal_update.type.data == 4:
                    # Update weight goal
                    self.manager.set_weight_goal(idx,goal_update.weight.data,t)
                else:
                    response.success = False
        else:
            response.success = False
        return response
