import yaml
import random
import lively_ik
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from wisc_msgs.msg import LivelyGoals
from wisc_actions.elements.structures import Position, Pose
from argparse import ArgumentParser
from lively_ik.groove.relaxed_ik_container import RelaxedIKContainer
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarker, InteractiveMarkerControl, Marker

class ControllerNode(Node):
    def __init__(self):
        super(ControllerNode,self).__init__('control')
        self.param_client = self.create_client(GetParameters, '/global_params/get_parameters')
        while not self.param_client.wait_for_service(1.0):
            if rclpy.ok():
                self.get_logger().info("Waiting for service to be available")
            else:
                exit()
        self.info = yaml.safe_load(self.get_lik_param('info'))

        self.ims = InteractiveMarkerServer(self,'control')
        self.goal_pub = self.create_publisher(LivelyGoals,'robot_goals',5)
        self.temp_js_pub = self.create_publisher(JointState,'joint_states',5)
        self.rik_container = RelaxedIKContainer(self.info)
        ee_positions = self.rik_container.robot.get_ee_positions(self.info['starting_config'])
        ee_rotations = self.rik_container.robot.get_ee_rotations(self.info['starting_config'])
        self.marker_lookup = []
        # Specify the ee_poses and configre markers
        ee_poses = []
        for ee_idx in range(len(self.info['joint_names'])):
            # Determine initial pose, and add the pose to the marker server.
            pos = ee_positions[ee_idx]
            rot = ee_rotations[ee_idx]
            name = self.info['ee_fixed_joints'][ee_idx]
            self.marker_lookup.append('interactive_marker_'+name)

            marker = self.create_marker(name,pos,rot,self.info['fixed_frame'])
            self.ims.insert(marker,feedback_callback=self.feedback_cb)

            # Set the initial value in self.goals
            pose_dict = {
                'position':{
                    'x':pos[0],
                    'y':pos[1],
                    'z':pos[2]
                },
                'orientation':{
                    'w':rot[0],
                    'x':rot[1],
                    'y':rot[2],
                    'z':rot[3]
                }
            }
            ee_poses.append(Pose.from_pose_dict(pose_dict).ros_pose)

        # Specify joint goals as initial
        dc_values = [Float64(data=float(d)) for d in self.info['starting_config']]

        # Specify the objective weights
        objective_weights =  []
        for obj in self.info['objectives']:
            objective_weights.append(Float64(data=float(obj['weight'])))

        # Specify the bias
        bias = Position(1.0,1.0,1.0).ros_point # UNIFORM
        # bias = Position(1.0,0.0,0.0).ros_point # LATERAL
        # bias = Position(0.0,1.0,0.0).ros_point # FRONT/BACK
        # bias = Position(0.0,0.0,1.0).ros_point # VERTICAL
        # bias = Position(1.0,0.3,2.0).ros_point # MIXED

        self.goals = LivelyGoals(ee_poses=ee_poses,dc_values=dc_values,objective_weights=objective_weights,bias=bias)

        self.ims.applyChanges()
        self.create_timer(0.05,self.publisher)
        js_msg = JointState(name=self.info['joint_ordering'],position=self.info['starting_config'])
        js_msg.header.stamp = self.get_clock().now().to_msg()
        self.temp_js_pub.publish(js_msg)
        self.destroy_publisher(self.temp_js_pub)

    def feedback_cb(self,msg):
        #self.get_logger().info("EE CB: {0} {1}".format(idx, msg))
        self.goals.ee_poses[self.marker_lookup.index(msg.marker_name)] = msg.pose

    def publisher(self):
        self.goals.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(self.goals)

    def get_lik_param(self, param):
        request = GetParameters.Request()
        request.names = [param]
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.values[0].string_value

    @staticmethod
    def create_marker(name,ee_pos,ee_rot,frame_id):
        # create an interactive marker for our server
        marker = InteractiveMarker()
        marker.header.frame_id = frame_id
        marker.name = "interactive_marker_" + name
        marker.pose.position.x = float(ee_pos[0])
        marker.pose.position.y = float(ee_pos[1])
        marker.pose.position.z = float(ee_pos[2])
        marker.pose.orientation.w = float(ee_rot[0])
        marker.pose.orientation.x = float(ee_rot[1])
        marker.pose.orientation.y = float(ee_rot[2])
        marker.pose.orientation.z = float(ee_rot[3])
        marker.scale = 0.2

        # create a grey box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.07
        box_marker.scale.y = 0.07
        box_marker.scale.z = 0.07
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 0.6

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)

        marker.controls.append(box_control)

        # Add Translation Controls
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_1"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_2"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_3"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        marker.controls.append(control)

        # Add Rotation Controls
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "rotate_1"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_2"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "rotate_3"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        marker.controls.append(control)

        return marker

def main():
    rclpy.init(args=None)

    node = ControllerNode()

    rclpy.spin(node)

    node.ims.shutdown()

if __name__ == '__main__':
    main()
