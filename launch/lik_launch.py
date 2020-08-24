from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions.launch_configuration import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml

def launch_setup(context, *args, **kwargs):
    lik_share = get_package_share_directory('lively_ik')
    config_name = LaunchConfiguration('config').perform(context)
    output_topic = LaunchConfiguration('output_topic').perform(context)
    config_path = lik_share + '/config/info_files/' + config_name

    rviz_file = lik_share + '/lik_viewer.rviz'

    with open(config_path,'r') as s:
        info_data = yaml.safe_load(s)
    info_string = yaml.dump(info_data)

    urdf_path = lik_share + '/config/urdfs/' + info_data['urdf_file_name']

    with open(urdf_path, 'r') as s:
        robot_desc = s.read()

    nodes = [
        # For debugging tf stuff
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0','0','0','0','0','0','world',info_data['fixed_frame']],
             output='screen'),
        # Node(name='joint_state_publisher',
        #      package='joint_state_publisher',
        #      executable='joint_state_publisher',
        #      output='screen'),
        Node(package='lively_ik',
             executable='param',
             parameters=[{'info':info_string},
                         {'output_topic':output_topic}]),
        # Node(package='lively_ik',
        #      executable='simple',
        #      output='screen'),
        Node(package='lively_ik',
             executable='control',
             output='screen'),
        Node(name='robot_state_publisher',
             package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_desc}]),
        Node(package='lively_ik',
             executable='Node.jl',
             output='screen'),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d',rviz_file])
    ]

    return nodes

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('config'),
        DeclareLaunchArgument('output_topic',default_value='/joint_states'),
        OpaqueFunction(function = launch_setup)
    ])
