from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions.launch_configuration import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml

def launch_setup(context, *args, **kwargs):
    lik_share = get_package_share_directory('lively_ik')
    urdf_name = LaunchConfiguration('urdf').perform(context)
    fixed_frame = LaunchConfiguration('fixed_frame').perform(context)
    urdf_path = lik_share + '/config/urdfs/' + urdf_name
    rviz_file = lik_share + '/lik_viewer.rviz'

    with open(urdf_path, 'r') as s:
        robot_desc = s.read()

    nodes = [
        # For debugging tf stuff
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0','0','0','0','0','0','world',fixed_frame],
             output='screen'),
        Node(name='robot_state_publisher',
             package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_desc}]),
        # Node(name='joint_state_publisher',
        #      package='joint_state_publisher',
        #      executable='joint_state_publisher',
        #      output='screen'),
        Node(name='joint_state_publisher_gui',
             package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             output='screen'),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d',rviz_file])
    ]

    return nodes

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('urdf'),
        DeclareLaunchArgument('fixed_frame'),
        OpaqueFunction(function = launch_setup)
    ])
