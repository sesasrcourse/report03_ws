from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import os

def generate_launch_description():

    pkg_name = 'r3pkg'
    task_number = '1'

    share_dir = get_package_share_directory(pkg_name)
    ws_dir = os.path.abspath(os.path.join(share_dir, '../../../../'))
    rosbags_dir = os.path.join(ws_dir, 'rosbags')
    if not os.path.exists(rosbags_dir):
        os.mkdir(rosbags_dir)
    
    timestamp = datetime.now().strftime("%d-%b_%H_%M_%S")
    bag = os.path.join(rosbags_dir, f'task_{task_number}_sim_{timestamp}')


    print(f"DIRECTORY OF THE ROSBAG: {bag}")

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'lab04.launch.py'
            ]),
        ),
        Node(
            package=pkg_name,
            executable='controller_node',
            name='controller_node',
            parameters=[
                {
                    'use_sim_time': True,
                }
            ]
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'record', '-a', '-o', bag],
        # )
    ])