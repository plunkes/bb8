"""Missão completa do BB8: explorar a arena por fronteiras e coletar a flag.

Atalho que encadeia os dois launches independentes:
  - simulation.launch.py  – Gazebo + mundo (prm_2026) + /sky_cam + /clock
  - spawn_robot.launch.py – robô + controladores + ponte + RViz + TODA a stack
                            (odom GT, scan_masker, SLAM, Nav2, explore, visão,
                            gripper, FSM)

Ou seja, rodar 'simulation.launch.py + spawn_robot.launch.py' equivale a rodar
este launch. Toda a lógica de nós/staging vive no spawn_robot.launch.py.

Árvore de TF resultante:
    map  -> odom        (slam_toolbox)
    odom -> base_link   (odom_gt_publisher, pose ground-truth do Gazebo)
    base_link -> sensores  (robot_state_publisher, TFs do URDF)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ctrl = FindPackageShare("bb8_control")

    # Mundo trocável via CLI: ros2 launch ... world:=arena_classic.sdf
    world_arg = DeclareLaunchArgument("world", default_value="arena_cilindros.sdf")

    # 1. Gazebo + mundo
    simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ctrl, "launch", "simulation.launch.py"])
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # 2. Robô + controladores + ponte + RViz + stack completa
    robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ctrl, "launch", "spawn_robot.launch.py"])
        ),
    )

    return LaunchDescription([world_arg, simulacao, robo])
