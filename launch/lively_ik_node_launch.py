from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
        # Node(
        #     package='lively_ik',
        #     namespace='lively_ik',
        #     executable='src/Node.jl',
        #     name='lively_ik_node'
        # )
    ])
