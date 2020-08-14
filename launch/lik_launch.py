from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions.launch_configuration import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml

def launch_setup(context, *args, **kwargs):
    lik_share = get_package_share_directory('lively_ik')
    config_name = LaunchConfiguration('config').perform(context)
    config_path = lik_share + '/config/info_files/' + config_name

    with open(config_path,'r') as s:
        info_data = yaml.safe_load(s)
    print(info_data)

    urdf_path = lik_share + '/config/urdfs/' + info_data['urdf_file_name']

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()


    nodes = [
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2'),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}]),
        Node(package='lively_ik',
             executable='Node.jl',
             parameters=[{'info':'some_value'}],
             output='screen')
    ]

    return nodes

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('config'),
        OpaqueFunction(function = launch_setup)
    ])
