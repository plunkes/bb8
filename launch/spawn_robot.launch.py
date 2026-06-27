"""Insere o robô BB8 no Gazebo e ativa o stack de controle.

Tudo é local ao bb8_control (descrição, controller_config e rviz migrados do
prm_2026):
  - robot_state_publisher (description/robot.urdf.xacro)
  - spawn da entidade no Gazebo (ros_gz_sim create)
  - ros2_control: joint_state_broadcaster -> diff_drive_base_controller ->
    gripper_controller (encadeados por OnProcessExit)
  - ponte ros_gz dos sensores do robô (LIDAR, IMU, câmera de segmentação, pose GT)
  - relay /cmd_vel -> /diff_drive_base_controller/cmd_vel_unstamped
  - RViz (rviz/rviz_config.rviz) — opcional via argumento 'rviz'
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ctrl = FindPackageShare("bb8_control")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Abrir o RViz com a config do BB8"
    )

    controller_config = PathJoinSubstitution(
        [pkg_ctrl, "config", "controller_config.yaml"]
    )

    # 1. robot_state_publisher (descrição local)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_ctrl, "launch", "robot_state_publisher.launch.py"]
            )
        )
    )

    # 2. Spawn da entidade no Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "prm_robot",
            "-topic", "robot_description",
            "-z", "1.0",
            "-x", "-8.0",
            "-y", "-0.5",
            "--ros-args", "--log-level", "warn",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # 3. ros2_control: broadcaster -> diff_drive -> gripper
    load_joint_state_broadcaster = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active", "joint_state_broadcaster",
        ],
        shell=False,
        output="screen",
    )
    start_diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_diff_drive_base_controller",
        arguments=["diff_drive_base_controller"],
        parameters=[controller_config],
        output="screen",
    )
    start_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        arguments=["gripper_controller"],
        parameters=[controller_config],
        output="screen",
    )

    # 4. Ponte dos sensores do robô (gz -> ros, read-only com '[')
    robot_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_prm_robot",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/robot_cam/labels_map@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/robot_cam/colored_map@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/robot_cam/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/model/prm_robot/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose",
        ],
        output="screen",
    )

    # 5. Relay de conveniência /cmd_vel -> controlador
    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diff_drive_base_controller/cmd_vel_unstamped",
            }
        ],
        output="screen",
    )

    # 6. RViz (opcional)
    rviz_config = PathJoinSubstitution([pkg_ctrl, "rviz", "rviz_config.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            rviz_arg,
            robot_bridge,
            rsp,
            spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[start_diff_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=start_diff_controller,
                    on_exit=[start_gripper_controller],
                )
            ),
            relay_cmd_vel,
            rviz_node,
        ]
    )
