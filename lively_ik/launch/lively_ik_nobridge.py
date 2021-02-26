from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node#, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
# from launch.actions import OpaqueFunction

def generate_launch_description():

    return LaunchDescription([
        #IncludeLaunchDescription(FrontendLaunchDescriptionSource(get_package_share_directory('rosbridge_server')+'/launch/rosbridge_websocket_launch.xml')),
        Node(name='robot_state_publisher',
             package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': '<?xml version="1.0" ?><robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"/><joint name="default_joint" type="fixed"><parent link="base_link" /><child link="default_link" /><origin xyz="0 0 0" rpy="0 0 0" /></joint><link name="default_link"/></robot>'}]),
        Node(name='lively_ik_interface',
             package='lively_ik',
             executable='interface',
             output='screen'),
        Node(name='lively_ik_solver',
             package='lively_ik',
             executable='solver',
             output='screen'),
        ])
