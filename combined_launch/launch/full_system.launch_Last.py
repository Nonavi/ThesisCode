# Este era sin rotar fisicamente la camara
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Argumento externo
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='fake',
        description='Type of robot: fake or real'
    )
    robot = LaunchConfiguration('robot')

    # Paths a los paquetes
    xarm_path = get_package_share_directory('xarm_moveit_config')
    octomap_path = get_package_share_directory('mi_octomap')
    realsense_path = get_package_share_directory('realsense2_camera')

    # Ruta al rosbag (ruta absoluta)
    bag_play_path = '/media/data/dockerRos2WS/bags/rosbag2_2025_04_18-10_20_37'

    return LaunchDescription([
        robot_arg,

        # 🦾 Xarm simulado
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(xarm_path, 'launch', 'xarm7_moveit_fake.launch.py')
            ),
            launch_arguments={'add_gripper': 'true'}.items(),
            condition=IfCondition(PythonExpression(["'", robot, "' == 'fake'"]))
        ),

        # 🤖 Xarm real
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(xarm_path, 'launch', 'xarm7_moveit_realmove.launch.py')
            ),
            launch_arguments={
                'robot_ip': '192.168.1.235',
                'add_gripper': 'true'
            }.items(),
            condition=IfCondition(PythonExpression(["'", robot, "' == 'real'"]))
        ),

        # 🧠 Octomap
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(octomap_path, 'launch', 'mi_octomap_rviz.launch.py')
            )
        ),

        # 📸 Realsense (solo en modo real)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_path, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true'
            }.items(),
            condition=IfCondition(PythonExpression(["'", robot, "' == 'real'"]))
        ),

        # 🔁 Bag playback (solo en modo fake)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_play_path, '--loop'],
            output='screen',
            condition=IfCondition(PythonExpression(["'", robot, "' == 'fake'"]))
        ),

        # 📍 Static TF para robot real: link_eef → camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_real',
            arguments=['0.05', '0', '0.07', '0', '-1.5708', '0', 'link_eef', 'camera_link'],
            output='screen',
            condition=IfCondition(PythonExpression(["'", robot, "' == 'real'"]))
        ),

        # 📍 Static TF para robot fake: world → camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_fake',
            arguments=['0.5', '0', '0.5', '0', '1.570796', '0', 'world', 'camera_link'],
            output='screen',
            condition=IfCondition(PythonExpression(["'", robot, "' == 'fake'"]))
        ),

        # 🎮 Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # 🕹️ DualSense teleop node
        Node(
            package='hello_moveit',
            executable='dual_sense_teleop_node',
            name='dual_sense_teleop_node',
            output='screen'
        ),

        # 🧠 Invocador node
        Node(
            package='hello_moveit',
            executable='invocador_node',
            name='invocador_node',
            output='screen'
        ),

        # 🧊 Cube intersection node
        Node(
            package='mi_octomap',
            executable='cube_intersection',
            name='cube_intersection_node',
            output='screen'
        ),

        # 📐 Paralelepípedo intersección
        Node(
            package='mi_octomap',
            executable='paralelepipedo_interseccion.py',
            name='paralelepipedo_interseccion_node',
            output='screen'
        ),

        # 🎯 Trigger set
        Node(
            package='mi_octomap',
            executable='triggers_set.py',
            name='triggers_set_node',
            output='screen'
        ),
    ])
