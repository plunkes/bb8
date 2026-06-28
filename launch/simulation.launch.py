"""Sobe o simulador Gazebo (Ignition) com o mundo da arena.

O MUNDO e os MODELOS continuam vindo do pacote prm_2026 (não foram modificados;
o bb8_control depende do prm_2026 apenas como provedor do ambiente de simulação).
Configura o IGN_GAZEBO_RESOURCE_PATH para os modelos do prm_2026 e cria a ponte
do mundo (/sky_cam e /clock).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Plugins do Gazebo (encontra os system plugins via LD_LIBRARY_PATH)
    gz_env = {
        "GZ_SIM_SYSTEM_PLUGIN_PATH": ":".join(
            [
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", default=""),
                os.environ.get("LD_LIBRARY_PATH", default=""),
            ]
        )
    }
    gz_verbosity = "3"

    world_file_arg = DeclareLaunchArgument(
        "world",
        default_value="arena_cilindros.sdf",
        description="Nome do arquivo .sdf do mundo (em prm_2026/world)",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="true = Gazebo SEM GUI (ign gazebo -s), p/ testes/CI sem display",
    )
    headless = LaunchConfiguration("headless")

    # Diretório de instalação do prm_2026 (provedor do mundo + modelos)
    pkg_prm = FindPackageShare("prm_2026").find("prm_2026")
    world_path = PathJoinSubstitution(
        [pkg_prm, "world", LaunchConfiguration("world")]
    )

    # GUI + servidor (uso normal, com display).
    gazebo_gui = ExecuteProcess(
        cmd=[
            "ruby", FindExecutable(name="ign"), "gazebo",
            "-r", "-v", gz_verbosity, world_path,
        ],
        output="screen",
        additional_env=gz_env,
        shell=False,
        condition=UnlessCondition(headless),
    )
    # Servidor apenas (-s): sem GUI. Evita que a GUI derrube o Gazebo quando não
    # há display (a GUI falha em criar a janela GL e mata o servidor junto).
    gazebo_server = ExecuteProcess(
        cmd=[
            "ruby", FindExecutable(name="ign"), "gazebo", "-s",
            "-r", "-v", gz_verbosity, world_path,
        ],
        output="screen",
        additional_env=gz_env,
        shell=False,
        condition=IfCondition(headless),
    )

    # Modelos personalizados do mundo: IGN_GAZEBO_RESOURCE_PATH -> prm_2026
    gz_models_path = ":".join([pkg_prm, os.path.join(pkg_prm, "models")])
    gz_set_env = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=gz_models_path,
    )

    # Ponte do mundo: câmera do céu + relógio simulado
    world_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_world",
        arguments=[
            "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription(
        [world_file_arg, headless_arg, gz_set_env, world_bridge, gazebo_gui, gazebo_server]
    )
