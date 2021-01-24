from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

DEFAULT_DESCRIPTION = '<?xml version="1.0" ?><robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"/><joint name="default_joint" type="fixed"><parent link="base_link" /><child link="default_link" /><origin xyz="0 0 0" rpy="0 0 0" /></joint><link name="default_link"/></robot>'

def launch_setup(context, *args, **kwargs):
    nodes = [
        Node(name='robot_state_publisher',
             package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': DEFAULT_DESCRIPTION}]),
        Node(name='lively_ik',
             package='lively_ik',
             executable='interface',
             output='screen')
    ]

    return nodes

def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])
