from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    xarm_path = get_package_share_directory('xarm_moveit_config')
    hello_path = get_package_share_directory('hello_moveit')
    octomap_path = get_package_share_directory('mi_octomap')

    return LaunchDescription([
        # Xarm real (MoveIt)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(xarm_path, 'launch', 'xarm7_moveit_realmove.launch.py')
            ),
            launch_arguments={
                'robot_ip': '192.168.1.235',
                'add_gripper': 'true'
            }.items()
        ),

        # Joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # DualSense teleop
        Node(
            package='hello_moveit',
            executable='dual_sense_teleop_node',
            name='dual_sense_teleop_node',
            output='screen'
        ),

        # Trigger set
        Node(
            package='mi_octomap',
            executable='triggers_set.py',
            name='triggers_set_node',
            output='screen'
        ),
    ])
