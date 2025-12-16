from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from datetime import datetime
import os

def set_up(context):

    pkg_name = 'r3pkg'

    obj_fun  = LaunchConfiguration('obj_fun').perform(context=context)

    config_file = LaunchConfiguration('conf', default='node_params_std.yaml').perform(context=context)

    share_dir = get_package_share_directory(pkg_name)
    ws_dir = os.path.abspath(os.path.join(share_dir, '../../../../'))
    rosbags_dir = os.path.join(ws_dir, 'rosbags')
    if not os.path.exists(rosbags_dir):
        os.mkdir(rosbags_dir)
    
    timestamp = datetime.now().strftime("%d-%b_%H_%M_%S")
    bag = os.path.join(rosbags_dir, f'r{obj_fun}_{timestamp}')


    print(f"DIRECTORY OF THE ROSBAG: {bag}")

    config_file_path = os.path.join(
        share_dir,
        'config',
        config_file
    )
    if not os.path.exists(config_file_path):
        raise FileNotFoundError
    print(f"CONFIG FILE: {config_file_path}")

    launch_config = [
        # ros2 launch turtlebot3_perception camera.launch.py            PathJoinSubstitution([
        IncludeLaunchDescription([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_perception'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        IncludeLaunchDescription([
            # ros2 launch turtlebot3_perception apriltag.launch.py
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_perception'),
                'launch',
                'apriltag.launch.py'
            ]),
        ]),
        Node(
            package=pkg_name,
            executable='controller_node',
            name='controller_node',
            parameters=[
                {
                    'obj_fun': obj_fun,
                    'use_sim_time': False,
                    'simulation': False,
                },
                config_file_path,
            ]
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '/ground_truth',
                '/scan',
                '/camera/landmarks',
                '/odom',
                '/cmd_vel',
                '/clock',
                '/rosout',
                '/goal_marker',
                '/dwa_feedback',
                '/filter_scan',
                '/dynamic_goal_pose',
                '/robot_description',
                '-o', bag
            ]
        )
    ]

    return launch_config

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('obj_fun', choices=['1', '2a', '2b'], description='Choose between objective functions 1, 2a, 2b'),
        DeclareLaunchArgument('conf', 
                              default_value='node_params_std.yaml', description='name of the config file to use for node params inside of the folder config'),
        OpaqueFunction(function=set_up)
    ])