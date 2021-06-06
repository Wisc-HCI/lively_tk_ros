#!/usr/bin/env python3
import rospy

import json
from copy import deepcopy

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Point
from tf2_web_republisher.msg import TFArray
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from configuration.config_manager import ConfigManager

from datetime import datetime

class Time(object):
    def __init__(self,sec,nanosec):
        self.sec = sec
        self.nanosec = nanosec

class InterfaceNode(object):
    def __init__(self):
        self.config_manager = ConfigManager(self.on_feedback,logger=lambda content:rospy.loginfo(str(content)))

        # Services and Topics
        self.js_pub = rospy.Publisher('/joint_states',JointState, queue_size=10)
        self.tf_pub = rospy.Publisher('tf_static',TFMessage,queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker',Marker,queue_size=10)

        self.gui_sub = rospy.Subscriber('/lively_ik/gui_updates', String, self.handle_gui_update, queue_size=10)
        self.gui_pub = rospy.Publisher('/lively_ik/server_updates', String, queue_size=10)
        self.tf_web_pub = rospy.Publisher('/lively_ik/tf_updates', TFArray, queue_size=10)
        self.tf_web_sub = rospy.Subscriber('/lively_ik/tf_request', String, self.handle_tf_request, queue_size=10)

        self.config_pub = rospy.Publisher('/lively_ik/config_updates', String, queue_size=10)
        self.weight_pub = rospy.Publisher('/lively_ik/weight_updates', String, queue_size=10)
        self.direct_pub = rospy.Publisher('/lively_ik/direct_updates', String, queue_size=10)
        self.enable_pub = rospy.Publisher('/lively_ik/enabled_updates', Bool, queue_size=10)
        self.refresh_pub = rospy.Publisher('/lively_ik/refresh_solver', String, queue_size=10)
        self.result_sub = rospy.Subscriber('/lively_ik/result', String, self.handle_result_update, queue_size=10)

        self.tf_broadcaster = TransformBroadcaster()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # self.file_writer = open('./p04/data_{0}.json'.format(datetime.now()),'a')

        self.watched_transforms = []
        self.marker_msgs = {}

        self.processes = []

        rospy.loginfo('Initialized!')

        self.displayed_state = [[0,0,0],[]]
        self.update_count = 0

    def on_feedback(self):
        rospy.loginfo('Reporting Feedback')
        rospy.loginfo(f'NN PROGRESS: {self.config_manager.meta["nn_progress"]}')
        self.pub_to_gui({'directive':'update','config':self.config_manager.data,'meta':self.config_manager.meta})

    def handle_tf_request(self,msg):
        rospy.logdebug('Received tf request')
        data = json.loads(msg.data)
        rospy.logdebug('{0}'.format(data))
        transform_pair = {'target':self.clean_tf_frame(data['target_frame']),
                          'source':self.clean_tf_frame(data['source_frame'])}
        if transform_pair not in self.watched_transforms:
            self.watched_transforms.append(transform_pair)

    def handle_result_update(self,msg):
        data = json.loads(msg.data)
        self.displayed_state = [data['base_transform'],data['joint_states']]
        if self.update_count > 5:
            changes = self.config_manager.update({},{'displayed_state':self.displayed_state},realtime_feedback=False)
            self.pub_to_gui({'directive':'update','meta':self.config_manager.meta})
            self.update_count = 0
        else:
            self.update_count += 1


    def handle_update(self,data):
        rospy.loginfo('Async update request')
        self.pub_to_gui({'directive':'update','meta':{'updating':True}})
        rospy.loginfo('{0}'.format(data))
        changes = self.config_manager.update(data.get('config',{}),data.get('meta',{}),realtime_feedback=False)
        self.file_writer.write(json.dumps({'time':str(datetime.now()),'config':data.get('config',{}),'meta':data.get('meta',{})})+',\n')
        self.pub_to_gui({'directive':'update','config':self.config_manager.data,'meta':self.config_manager.meta})
        if 'urdf' in changes and self.config_manager.meta['valid_urdf']:
            rospy.loginfo('Updating robot description!')
            self.update_robot_description(self.config_manager.data['urdf'])
            self.watched_transforms = []
            for link in self.config_manager.meta['links']:
                transform_pair = {'target':self.clean_tf_frame('world'),
                                  'source':self.clean_tf_frame(link)}
                if transform_pair not in self.watched_transforms:
                    self.watched_transforms.append(transform_pair)
            rospy.loginfo('Updated robot description!')
        if 'objectives' in changes and self.config_manager.meta['valid_config']:
            self.config_pub.publish(String(data=json.dumps(self.config_manager.data)))
        self.pub_to_gui({'directive':'update','meta':{'updating':False}})
        self.update_markers()
        rospy.loginfo('Handled update request')

    def handle_default_update(self,data):
        self.pub_to_gui({'directive':'update','config':self.config_manager.data,'meta':self.config_manager.meta})
        self.update_markers()
        rospy.loginfo('Handled default request')

    def handle_train_nn(self,_):
        self.pub_to_gui({'directive':'update','meta':{'updating':True}})
        self.config_manager.train_nn()
        self.pub_to_gui({'directive':'update','meta':{'updating':False}})
        rospy.loginfo('{0}'.format(self.config_manager.current['config']))
        rospy.loginfo('Handled train request')

    def handle_clear(self,_):
        self.pub_to_gui({'directive':'update','meta':{'updating':True}})
        self.config_manager = ConfigManager(self.on_feedback,logger=lambda content:rospy.loginfo(str(content)))
        self.update_markers()
        rospy.loginfo('Handled clear request')

    def handle_refresh(self,_):
        self.refresh_pub.publish(String(data=json.dumps(self.config_manager.data)))

    def handle_gui_update(self,msg):
        rospy.loginfo('Received update request')
        data = json.loads(msg.data)

        if data['directive'] == 'update':
            self.dispatch_async(self.handle_update,data)
        elif data['directive'] == 'process' and data['type'] == 'nn':
            self.dispatch_async(self.handle_train_nn,data)
        elif data['directive'] == 'clear':
            self.dispatch_async(self.handle_clear,data)
        elif data['directive'] == 'refresh':
            self.dispatch_async(self.handle_refresh,data)
        else:
            self.dispatch_async(self.handle_default_update,data)

        rospy.loginfo('Dispatched requests')

    def update_markers(self):
        current_markers = self.config_manager.current['markers']
        rospy.loginfo(str(current_markers.keys()))
        new_markers = {}
        idx = 0
        # First handle the existing markers
        for name,marker in self.marker_msgs.items():
            if name not in current_markers:
                # Delete the marker, since it is no longer needed
                marker.action = marker.DELETE
                self.marker_pub.publish(marker)
            else:
                # Update the marker, in case it changed
                current_marker = current_markers[name]
                marker.header.frame_id = current_marker['frame_id']
                marker.id = idx
                marker.pose.position.x = float(current_marker['pose']['position']['x'])
                marker.pose.position.y = float(current_marker['pose']['position']['y'])
                marker.pose.position.z = float(current_marker['pose']['position']['z'])
                marker.pose.orientation.w = float(current_marker['pose']['orientation']['w'])
                marker.pose.orientation.x = float(current_marker['pose']['orientation']['x'])
                marker.pose.orientation.y = float(current_marker['pose']['orientation']['y'])
                marker.pose.orientation.z = float(current_marker['pose']['orientation']['z'])
                marker.scale.x = float(current_marker['scale']['x'])
                marker.scale.y = float(current_marker['scale']['y'])
                marker.scale.z = float(current_marker['scale']['z'])
                marker.color.a = float(current_marker['color']['a'])
                marker.color.r = float(current_marker['color']['r'])
                marker.color.g = float(current_marker['color']['g'])
                marker.color.b = float(current_marker['color']['b'])
                if current_marker['type'] == 'arrow':
                    marker.type = marker.ARROW
                elif current_marker['type'] == 'sphere':
                    marker.type = marker.SPHERE
                elif current_marker['type'] == 'cube':
                    marker.type = marker.CUBE
                elif current_marker['type'] == 'cylinder':
                    marker.type = marker.CYLINDER
                elif current_marker['type'] == 'points':
                    marker.type = marker.POINTS
                    marker.points = [Point(x=point['x'],y=point['y'],z=point['z']) for point in current_marker['data']]
                elif current_marker['type'] == 'text':
                    marker.type = marker.TEXT_VIEW_FACING
                    marker.text = current_marker['data']
                else:
                    marker.type = marker.MESH_RESOURCE
                    marker.mesh_resource = current_marker['type']
                marker.action = marker.MODIFY
                new_markers[name] = marker
                self.marker_pub.publish(marker)
                idx += 1
        # Now add any new markers not present in existing
        for name,current_marker in current_markers.items():
            if name not in self.marker_msgs:
                marker = Marker()
                marker.header.frame_id = current_marker['frame_id']
                marker.id = idx
                marker.pose.position.x = float(current_marker['pose']['position']['x'])
                marker.pose.position.y = float(current_marker['pose']['position']['y'])
                marker.pose.position.z = float(current_marker['pose']['position']['z'])
                marker.pose.orientation.w = float(current_marker['pose']['orientation']['w'])
                marker.pose.orientation.x = float(current_marker['pose']['orientation']['x'])
                marker.pose.orientation.y = float(current_marker['pose']['orientation']['y'])
                marker.pose.orientation.z = float(current_marker['pose']['orientation']['z'])
                marker.scale.x = float(current_marker['scale']['x'])
                marker.scale.y = float(current_marker['scale']['y'])
                marker.scale.z = float(current_marker['scale']['z'])
                marker.color.a = float(current_marker['color']['a'])
                marker.color.r = float(current_marker['color']['r'])
                marker.color.g = float(current_marker['color']['g'])
                marker.color.b = float(current_marker['color']['b'])
                if current_marker['type'] == 'arrow':
                    marker.type = marker.ARROW
                elif current_marker['type'] == 'sphere':
                    marker.type = marker.SPHERE
                elif current_marker['type'] == 'cube':
                    marker.type = marker.CUBE
                elif current_marker['type'] == 'cylinder':
                    marker.type = marker.CYLINDER
                elif current_marker['type'] == 'points':
                    marker.type = marker.POINTS
                    marker.points = [Point(x=point['x'],y=point['y'],z=point['z']) for point in current_marker['data']]
                elif current_marker['type'] == 'text':
                    marker.type = marker.TEXT_VIEW_FACING
                    marker.text = current_marker['data']
                else:
                    marker.type = marker.MESH_RESOURCE
                    marker.mesh_resource = current_marker['type']
                marker.action = marker.ADD
                new_markers[name] = marker
                self.marker_pub.publish(marker)
                idx += 1
        self.marker_msgs = new_markers

    def dispatch_async(self,fn,data={}):
        self.processes.append({'fn':fn,'data':data})

    def standard_loop(self):
        while len(self.processes) > 0:
            process = self.processes.pop(0)
            process['fn'](process['data'])

        data = self.config_manager.data
        meta = self.config_manager.meta

        if meta['valid_solver'] and meta['control'] == 'solve':
            self.set_solver_activity(meta['control'] == 'solve')
            self.weight_pub.publish(String(data=json.dumps(meta['target_weights'])))
            self.direct_pub.publish(String(data=json.dumps(meta['target_goals'])))
        elif meta['valid_urdf']:
            # Set the transform to the middle of the allowed space
            self.displayed_state = meta['displayed_state']

        # Send the transform for the base
        t = TransformStamped()
        t.header.stamp = rospy.get_rostime()
        t.header.frame_id = "world"
        t.child_frame_id = data['fixed_frame']
        t.transform.translation.x = float(self.displayed_state[0][0])
        t.transform.translation.y = float(self.displayed_state[0][1])
        t.transform.translation.z = float(self.displayed_state[0][2])
        self.tf_broadcaster.sendTransform(t)

        # Send the joint states
        js = JointState(name=data['joint_ordering'],position=self.displayed_state[1])
        js.header.stamp = rospy.get_rostime()
        self.js_pub.publish(js)

        transforms = []
        # rospy.loginfo(f'watched_transforms: {self.watched_transforms}')
        for pair in self.watched_transforms:
            try:
                transform = self.tf_buffer.lookup_transform(pair['target'],pair['source'],Time(0,0))
                transform.header.stamp = rospy.get_rostime()
                transform.header.frame_id = pair['target']
                transform.child_frame_id = pair['source']
                transforms.append(transform)
            except Exception as e:
                pass#rospy.loginfo('{0}'.format(e))
        # rospy.loginfo('{0}'.format(transforms))
        self.tf_web_pub.publish(TFArray(transforms=transforms))

    def pub_to_gui(self,data):
        data_msg = String(data=json.dumps(data))
        self.gui_pub.publish(data_msg)

    def set_solver_activity(self,enabled):
        self.enable_pub.publish(Bool(data=enabled))

    def update_robot_description(self,urdf):
        rospy.set_param('robot_description',urdf)

    @staticmethod
    def clean_tf_frame(frame_id:str) -> str:
        return frame_id.replace('/','')

def main():
    rospy.init_node('lively_ik_interface')

    node = InterfaceNode()
    while not rospy.is_shutdown():
        node.standard_loop()

    

if __name__ == '__main__':
    main()
