from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_path = get_package_share_directory('realsense2_camera')
    octomap_path = get_package_share_directory('mi_octomap')
    hello_path = get_package_share_directory('hello_moveit')

    return LaunchDescription([
        # Cámara Realsense
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_path, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
                'use_intra_process_comms': 'false'
            }.items()
        ),

        # Octomap completo (igual que en robot=real)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(octomap_path, 'launch', 'mi_octomap_rviz.launch.py')
            )
        ),

        # Static TF: link_eef → camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_real',
            arguments=['0.05', '0', '0.07', '0', '-1.5708', '3.1416', 'link_eef', 'camera_link'],
            output='screen'
        ),

        # Cube intersection
        Node(
            package='mi_octomap',
            executable='cube_intersection',
            name='cube_intersection_node',
            output='screen'
        ),

        # Paralelepípedo intersección
        Node(
            package='mi_octomap',
            executable='paralelepipedo_interseccion.py',
            name='paralelepipedo_interseccion_node',
            output='screen'
        ),
    ])
