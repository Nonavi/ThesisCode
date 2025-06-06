from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch.events.process import ProcessStarted, SignalProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    xarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("xarm_moveit_config").find("xarm_moveit_config"),
                "launch",
                "xarm7_moveit_realmove.launch.py"
            )
        ),
        launch_arguments={
            "robot_ip": "192.168.1.235",
            "add_gripper": "true"
        }.items()
    )

    pub_pose = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "-r", "10", "/rviz/moveit/move_marker/goal_link_tcp",
            "geometry_msgs/msg/PoseStamped",
            "{header: {frame_id: 'world'}, pose: {position: {x: 0.02, y: -0.24, z: 0.31}, orientation: {x: -0.68, y: 0.73, z: -0.01, w: 0.02}}}"
        ],
        shell=True,
        name="pose_publisher",
        output="screen"
    )

    stop_pub = RegisterEventHandler(
        OnProcessStart(
            target_action=pub_pose,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[
                        EmitEvent(event=SignalProcess(
                            signal_number='SIGINT',
                            process_matcher=lambda proc: proc.action == pub_pose
                        ))
                    ]
                )
            ]
        )
    )

    return LaunchDescription([
        xarm_launch,
        TimerAction(period=3.0, actions=[pub_pose]),
        stop_pub
    ])
