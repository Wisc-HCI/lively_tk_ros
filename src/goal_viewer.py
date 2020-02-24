#! /usr/bin/env python
'''
author: Andrew Schoen
website: http://pages.cs.wisc.edu/~schoen/
email: schoen@cs.wisc.edu
last update: 02/18/2020

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''
######################################################################################################

from visualization_msgs.msg import Marker, MarkerArray
import rospy
import tf
from wisc_msgs.msg import EEPoseGoals


class MarkerViewer(object):
    def __init__(self):
        self.fixed_frame = rospy.get_param('fixed_frame')
        self.fixed_joints = rospy.get_param('ee_fixed_joints')
        self.markers = {}
        for i,fixed_joint in enumerate(self.fixed_joints):
            self.markers[fixed_joint] = Marker()
            self.markers[fixed_joint].header.frame_id = self.fixed_frame
            self.markers[fixed_joint].action = Marker.ADD
            self.markers[fixed_joint].type = Marker.ARROW
            self.markers[fixed_joint].id = i
            self.markers[fixed_joint].scale.x = 0.1
            self.markers[fixed_joint].scale.y = 0.03
            self.markers[fixed_joint].scale.z = 0.01
            self.markers[fixed_joint].color.a = 1.0
            self.markers[fixed_joint].color.r = 0.75
            self.markers[fixed_joint].color.g = 0.75
            self.markers[fixed_joint].color.b = 0.1

        self.js_sub = rospy.Subscriber('/relaxed_ik/ee_pose_goals',EEPoseGoals, self.pg_sub_cb, queue_size=5)
        self.marker_pub = rospy.Publisher('/visualization_marker',Marker, queue_size=5)
        self.tf_pub = tf.TransformBroadcaster()


    def pg_sub_cb(self,pg_msg):
        for i,pose in enumerate(pg_msg.ee_poses):
            marker = self.markers[self.fixed_joints[i]]
            marker.header.stamp = rospy.Time.now()
            marker.pose = pose
            self.marker_pub.publish(marker)

        # Update transform
        self.tf_pub.sendTransform((0,0,0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             'common_world',
                             self.fixed_frame)


if __name__ == '__main__':
    rospy.init_node('goal_viewer')

    try:
        viewer = MarkerViewer()
    except:
        rospy.logerr('Could not retrieve/apply required parameters!')
        exit()

    while not rospy.is_shutdown():
        rospy.spin()
