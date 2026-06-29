"""Missão completa do BB8: explorar a arena por fronteiras e coletar a flag.

Stack: Gazebo + robô + odometria GT + SLAM + Nav2 + exploração de fronteiras
(m-explore / explore_lite) + visão (detecção da bandeira) + FSM orquestradora.

Árvore de TF resultante:
    map  -> odom        (slam_toolbox)
    odom -> base_link   (odom_gt_publisher, pose ground-truth do Gazebo)
    base_link -> sensores  (robot_state_publisher, TFs do URDF)

Ordem de subida (escalonada p/ SLAM/Nav2 prontos antes de explore/FSM):
  1. simulation        – Gazebo + mundo (prm_2026) + /sky_cam + /clock
  2. spawn_robot       – RSP, spawn, controladores, ponte dos sensores, RViz
  3. odom_gt_publisher – TF 'odom' -> base_link (nó do bb8_control)
  4. scan_masker       – mascara o setor frontal do LIDAR quando o braço sobe
  5. slam_toolbox      – mapeamento online (/map, TF map->odom)
  6. nav2              – navigation_launch.py
  7. explore_lite      – exploração de fronteiras (FSM controla via explore/resume)
  8. vision_processor  – detecção semântica da bandeira
  9. gripper_server    – serviço do gripper (postura inicial retraída)
 10. controle_robo     – FSM orquestradora
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_ctrl = FindPackageShare("bb8_control")
    pkg_nav2 = FindPackageShare("nav2_bringup")

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

    # Mundo trocável via CLI: ros2 launch ... world:=arena_classic.sdf
    world_arg = DeclareLaunchArgument("world", default_value="arena_cilindros.sdf")

    # 1. Gazebo + mundo
    simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ctrl, "launch", "simulation.launch.py"])
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # 2. Robô + controladores + ponte + RViz
    robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ctrl, "launch", "spawn_robot.launch.py"])
        ),
    )

    # 3. Odometria ground-truth (TF odom -> base_link) — nó do bb8_control
    gt_odom = Node(
        package="bb8_control",
        executable="odom_gt_publisher",
        name="ground_truth_odometry",
        parameters=[{"use_sim_time": True, "odom_frame": "odom"}],
        output="log",  # logs no arquivo; só o controle_robo vai p/ a tela
    )

    # 4. Filtro dinâmico do LIDAR (publica /scan_filtered p/ SLAM e Nav2)
    scan_masker = Node(
        package="bb8_control",
        executable="scan_masker",
        name="scan_masker",
        parameters=[{"use_sim_time": True}],
        output="log",
    )

    # 5. SLAM
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params, {"use_sim_time": True}],
        output="log",
    )

    # 6. Nav2
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

    # 7. Exploração de fronteiras (m-explore / explore_lite)
    explore = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        parameters=[explore_params, {"use_sim_time": True}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="log",
    )

    # 8. Visão (detecção da bandeira)
    visao = Node(
        package="bb8_control",
        executable="vision_processor",
        name="vision_processor",
        parameters=[
            {"use_sim_time": True},
            {"flag_label_ids": [25]},
            {"camera_hfov_deg": 108.86},  # casa com horizontal_fov 1.9 rad do sensor
        ],
        output="log",
    )

    # 9. Servidor do gripper
    gripper = Node(
        package="bb8_control",
        executable="gripper_server",
        name="gripper_server",
        parameters=[{"use_sim_time": True}],
        output="log",
    )

    # 10. FSM orquestradora
    controle = Node(
        package="bb8_control",
        executable="controle_robo",
        name="controle_robo",
        parameters=[fsm_params, {"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            simulacao,
            robo,
            gt_odom,
            TimerAction(period=5.0, actions=[scan_masker]),
            TimerAction(period=6.0, actions=[slam]),
            TimerAction(period=9.0, actions=[nav2]),
            TimerAction(period=8.0, actions=[gripper]),
            TimerAction(period=13.0, actions=[explore, visao, controle]),
        ]
    )
