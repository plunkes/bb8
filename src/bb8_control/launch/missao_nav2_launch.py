"""Missão completa com stack moderna: Gazebo + robô + SLAM + Nav2 + m-explore + FSM.

Árvore de TF resultante:
    map  -> odom        (slam_toolbox)
    odom -> base_link   (ground_truth_odometry, pose ground-truth do Gazebo)
    base_link -> sensores  (robot_state_publisher, TFs estáticos do URDF)

Ordem de subida (escalonada para o SLAM/Nav2 estarem prontos antes do explore/FSM):
  1. inicia_simulacao   – Gazebo com arena_cilindros.sdf
  2. carrega_robo       – robot_state_publisher, spawn, controladores, bridges, RViz
  3. ground_truth_odometry – TF/odom 'odom' -> base_link
  4. slam_toolbox       – mapeamento online (/map, TF map->odom)
  5. nav2               – navigation_launch.py (planner, controller, bt_navigator, ...)
  6. explore_lite       – exploração de fronteiras (controlada pela FSM via explore/resume)
  7. vision_processor   – detecção semântica da bandeira
  8. controle_robo      – FSM orquestradora
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim = FindPackageShare("prm_2026")
    pkg_ctrl = FindPackageShare("bb8_control")
    pkg_nav2 = FindPackageShare("nav2_bringup")

    slam_params = PathJoinSubstitution([pkg_ctrl, "config", "slam_toolbox.yaml"])
    nav2_params = PathJoinSubstitution([pkg_ctrl, "config", "nav2_params.yaml"])
    explore_params = PathJoinSubstitution([pkg_ctrl, "config", "explore_params.yaml"])

    # 1. Gazebo com o mundo da arena
    simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "inicia_simulacao.launch.py"])
        ),
        launch_arguments={"world": "arena_cilindros.sdf"}.items(),
    )

    # 2. Robô + controladores + bridges + RViz
    robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "carrega_robo.launch.py"])
        ),
    )

    # 3. Odometria ground-truth (publica TF odom -> base_link)
    gt_odom = Node(
        package="prm_2026",
        executable="ground_truth_odometry",
        name="ground_truth_odometry",
        parameters=[{"use_sim_time": True, "odom_frame": "odom"}],
        output="screen",
    )

    # 4. SLAM (mapeamento online assíncrono)
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params, {"use_sim_time": True}],
        output="screen",
    )

    # 5. Nav2 (sem amcl/map_server: mapa vem do slam_toolbox)
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

    # 6. Exploração de fronteiras (m-explore). A FSM controla via explore/resume.
    explore = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        parameters=[explore_params, {"use_sim_time": True}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="screen",
    )

    # 7. Processamento visual da bandeira
    visao = Node(
        package="bb8_control",
        executable="vision_processor",
        name="vision_processor",
        parameters=[{"use_sim_time": True}, {"flag_label_ids": [25]}],
        output="screen",
    )

    # 8. FSM orquestradora
    controle = Node(
        package="bb8_control",
        executable="controle_robo",
        name="controle_robo",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            simulacao,
            robo,
            gt_odom,
            # SLAM após o robô/controladores subirem
            TimerAction(period=6.0, actions=[slam]),
            # Nav2 depois do SLAM publicar map->odom
            TimerAction(period=9.0, actions=[nav2]),
            # Explore + visão + FSM por último (FSM espera o action server do Nav2)
            TimerAction(period=13.0, actions=[explore, visao, controle]),
        ]
    )
