from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from datetime import datetime
import os

def set_up(context):

    pkg_name = 'r3pkg'

    obj_fun  = LaunchConfiguration('obj_fun').perform(context=context)

    share_dir = get_package_share_directory(pkg_name)
    ws_dir = os.path.abspath(os.path.join(share_dir, '../../../../'))
    rosbags_dir = os.path.join(ws_dir, 'rosbags')
    if not os.path.exists(rosbags_dir):
        os.mkdir(rosbags_dir)
    
    timestamp = datetime.now().strftime("%d-%b_%H_%M_%S")
    bag = os.path.join(rosbags_dir, f'r{obj_fun}_{timestamp}')


    print(f"DIRECTORY OF THE ROSBAG: {bag}")

    launch_config = [
        Node(
            package=pkg_name,
            executable='controller_node',
            name='controller_node',
            parameters=[
                {
                    'obj_fun': obj_fun,
                    'simulation': False,
                }
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
        OpaqueFunction(function=set_up)
    ])