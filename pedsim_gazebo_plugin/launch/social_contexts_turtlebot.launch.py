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
    world_file_name = 'social_contexts_turtlebot.world'
    world_file = os.path.join(get_package_share_directory('pedsim_gazebo_plugin'), 'worlds', world_file_name)
    # launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # urdf_file_name = 'turtlebot3_burger.urdf'

    # print('urdf_file_name : {}'.format(urdf_file_name))

    # urdf = os.path.join(
    #     get_package_share_directory('turtlebot3_description'),
    #     'urdf',
    #     urdf_file_name)

    # pedsim_gazebo_plugin_path = get_package_share_directory('pedsim_gazebo_plugin')
    
    gzserver_launch_py = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file, 
            'use_sim_time': 'true',
            'gdb': 'false',
        }.items()
    )

    gzclient_launch_py = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            )
        ),
        launch_arguments={
            # 'use_sim_time': True,
            # 'world': world_file,
            "gdb": 'false'
        }.items()
    )

    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
        #     output='screen'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        gzserver_launch_py,

        gzclient_launch_py,
        # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
        # Node(package='gazebo_ros', node_executable='spawn_entity.py',
        #     arguments=['-entity', 'demo', '-database', 'double_pendulum_with_base'],
        #     output='screen')
        Node(
            package='pedsim_gazebo_plugin',
            executable='spawn_pedsim_agents.py',
            name='spawn_pedsim_agents',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])