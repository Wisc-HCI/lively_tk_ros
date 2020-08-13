from launch import LaunchDescription, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import yaml

def launch_setup(context, *args, **kwargs):
    lik_share = get_package_share_directory('lively_ik')
    config_name = launch.substitutions.LaunchConfiguration('config').perform(context)
    config_path = lik_share + '/config/info_files/' + config_name
    with open(config_path,'r') as s:
        info_data = yaml.load(s)
    print(info_data)


    with open(info_data['urdf_file_name'], 'r') as infp:
        robot_desc = infp.read()


    nodes = [
        Node(package='rviz2',
             namespace='rviz2',
             executable='rviz2',
             name='rviz2'),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}]),
        Node(package='lively_ik',
             namespace='lively_ik',
             executable='/Node.jl',
             name='lively_ik_node',
             parameters=[info_data],
             output='screen')
    ]

    return nodes

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('config'),
        OpaqueFunction(function = launch_setup)
    ])
