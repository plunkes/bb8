"""Insere o robô BB8 no Gazebo e sobe TODA a stack de controle/navegação.

Este launch é o par do simulation.launch.py: rodando
    simulation.launch.py + spawn_robot.launch.py
sobe exatamente o mesmo conjunto de nós do explore_and_catch_flag.launch.py
(que agora apenas encadeia esses dois).

Conteúdo:
  - robot_state_publisher (description/robot.urdf.xacro)
  - spawn da entidade no Gazebo (ros_gz_sim create)
  - ros2_control: joint_state_broadcaster -> diff_drive_base_controller ->
    gripper_controller (encadeados por OnProcessExit)
  - ponte ros_gz dos sensores do robô (LIDAR, IMU, câmera de segmentação, pose GT)
  - relay /cmd_vel -> /diff_drive_base_controller/cmd_vel_unstamped
  - odom_gt_publisher, scan_masker (nós do bb8_control)
  - SLAM (slam_toolbox) + Nav2 (navigation_launch.py)
  - explore_lite (exploração de fronteiras)
  - vision_processor, gripper_server, controle_robo (nós do bb8_control)
  - RViz (rviz/rviz_config.rviz) — opcional via argumento 'rviz'

Política de logs: nós de pacotes EXTERNOS (ros_gz_*, controller_manager, rviz2,
robot_state_publisher, topic_tools, slam_toolbox, explore_lite) usam
output="log". Só os nós do pacote bb8_control usam output="screen", então na
tela aparecem apenas os logs do bb8_control. (Nav2 é um include e gerencia o
próprio output.)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_ctrl = FindPackageShare("bb8_control")
    pkg_nav2 = FindPackageShare("nav2_bringup")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Abrir o RViz com a config do BB8"
    )

    controller_config = PathJoinSubstitution(
        [pkg_ctrl, "config", "controller_config.yaml"]
    )
    slam_params = PathJoinSubstitution([pkg_ctrl, "config", "slam_toolbox.yaml"])
    nav2_params_src = PathJoinSubstitution([pkg_ctrl, "config", "nav2_params.yaml"])
    explore_params = PathJoinSubstitution([pkg_ctrl, "config", "explore_params.yaml"])
    fsm_params = PathJoinSubstitution([pkg_ctrl, "config", "fsm_params.yaml"])

    # Injeta o caminho da BT customizada (sem ré) no nav2_params em runtime.
    bt_no_backup = PathJoinSubstitution(
        [pkg_ctrl, "behavior_trees", "navigate_no_backup.xml"]
    )
    nav2_params = RewrittenYaml(
        source_file=nav2_params_src,
        param_rewrites={"default_nav_to_pose_bt_xml": bt_no_backup},
        convert_types=True,
    )

    # 1. robot_state_publisher (descrição local) — externo
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_ctrl, "launch", "robot_state_publisher.launch.py"]
            )
        )
    )

    # 2. Spawn da entidade no Gazebo — externo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
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

    # 3. ros2_control: broadcaster -> diff_drive -> gripper — externo
    load_joint_state_broadcaster = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active", "joint_state_broadcaster",
        ],
        shell=False,
        output="log",
    )
    start_diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_diff_drive_base_controller",
        arguments=["diff_drive_base_controller"],
        parameters=[controller_config],
        output="log",
    )
    start_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        arguments=["gripper_controller"],
        parameters=[controller_config],
        output="log",
    )

    # 4. Ponte dos sensores do robô (gz -> ros, read-only com '[') — externo
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
        output="log",
    )

    # 5. Relay de conveniência /cmd_vel -> controlador — externo
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
        output="log",
    )

    # 6. RViz (opcional) — externo
    rviz_config = PathJoinSubstitution([pkg_ctrl, "rviz", "rviz_config.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config],
    )

    # 7. Odometria ground-truth (TF odom -> base_link) — nó do bb8_control
    gt_odom = Node(
        package="bb8_control",
        executable="odom_gt_publisher",
        name="ground_truth_odometry",
        parameters=[{"use_sim_time": True, "odom_frame": "odom"}],
        output="screen",
    )

    # 8. Filtro dinâmico do LIDAR (/scan_filtered p/ SLAM e Nav2) — bb8_control
    scan_masker = Node(
        package="bb8_control",
        executable="scan_masker",
        name="scan_masker",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 9. SLAM — externo
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params, {"use_sim_time": True}],
        output="log",
    )

    # 10. Nav2 — include (gerencia o próprio output)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2, "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "params_file": nav2_params,
        }.items(),
    )

    # 11. Exploração de fronteiras (m-explore / explore_lite) — externo
    explore = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        parameters=[explore_params, {"use_sim_time": True}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="log",
    )

    # 12. Visão (detecção da bandeira) — nó do bb8_control
    visao = Node(
        package="bb8_control",
        executable="vision_processor",
        name="vision_processor",
        parameters=[
            {"use_sim_time": True},
            {"flag_label_ids": [25]},
            {"camera_hfov_deg": 108.86},  # casa com horizontal_fov 1.9 rad do sensor
        ],
        output="screen",
    )

    # 13. Servidor do gripper — nó do bb8_control
    gripper_server = Node(
        package="bb8_control",
        executable="gripper_server",
        name="gripper_server",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 14. FSM orquestradora — nó do bb8_control
    controle = Node(
        package="bb8_control",
        executable="controle_robo",
        name="controle_robo",
        parameters=[fsm_params, {"use_sim_time": True}],
        output="screen",
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
            gt_odom,
            TimerAction(period=5.0, actions=[scan_masker]),
            TimerAction(period=6.0, actions=[slam]),
            TimerAction(period=8.0, actions=[gripper_server]),
            TimerAction(period=9.0, actions=[nav2]),
            TimerAction(period=13.0, actions=[explore, visao, controle]),
        ]
    )
