import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # world_file_name = 'turtlebot3_worlds/' + TURTLEBOT3_MODEL + '.model'
    world_file_name = 'social_contexts.world'
    world = os.path.join(get_package_share_directory('pedsim_gazebo_plugin'), 'worlds', world_file_name)
    # launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # urdf_file_name = 'turtlebot3_burger.urdf'

    # print('urdf_file_name : {}'.format(urdf_file_name))

    # urdf = os.path.join(
    #     get_package_share_directory('turtlebot3_description'),
    #     'urdf',
    #     urdf_file_name)

    # pedsim_gazebo_plugin_path = get_package_share_directory('pedsim_gazebo_plugin')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
            output='screen'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=[urdf]
        # ),
    ])