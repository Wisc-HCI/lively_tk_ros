from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory

DEFAULT_DESCRIPTION = '''
<?xml version="1.0" ?>
<robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <link name="base_link"/>
    <joint name="default_joint" type="fixed">
        <parent link="base_link" />
        <child link="default_link" />
        <origin xyz="0 0 0" rpy="0 0 0" />
    </joint>
    <link name="default_link"/>
</robot>
'''

def launch_setup(context, *args, **kwargs):
    lik_share = get_package_share_directory('lively_ik')
    rviz_file = lik_share + '/lik_viewer.rviz'
    nodes = [
        # Node(package='tf2_ros',
        #      executable='static_transform_publisher',
        #      arguments=['0','0','0','0','0','0','world','base_link'],
        #      output='screen'),
        Node(name='robot_state_publisher',
             package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': DEFAULT_DESCRIPTION}]),
        Node(name='lively_apps',
             package='lively_ik',
             executable='interface',
             output='screen'),
        # Node(package='rviz2',
        #      executable='rviz2',
        #      name='rviz2',
        #      arguments=['-d',rviz_file])
    ]

    return nodes

def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])
