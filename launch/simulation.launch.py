"""Sobe o simulador Gazebo (Ignition) com o mundo da arena.

O MUNDO e os MODELOS continuam vindo do pacote prm_2026 (não foram modificados;
o bb8_control depende do prm_2026 apenas como provedor do ambiente de simulação).
Configura o IGN_GAZEBO_RESOURCE_PATH para os modelos do prm_2026 e cria a ponte
do mundo (/sky_cam e /clock).

Este launch cuida APENAS do mundo simulado no Gazebo. O robô (RSP, spawn,
controladores, RViz) fica no spawn_robot.launch.py.

Passo da simulação: como o .sdf do prm_2026 não é modificado, o max_step_size é
aumentado em runtime via 'ign service .../set_physics' (só launch files), o que
faz a simulação rodar mais rápido.
"""

import os
import re

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Passo de integração da física (s). Maior => menos iterações por segundo de
# simulação => roda mais rápido. Default do Ignition é 0.001; aqui 0.005 (5x).
SIM_MAX_STEP_SIZE = 0.005
# RTF alvo. NÃO usar valores enormes: SLAM/tf/Nav2 são assíncronos e não seguem
# um /clock disparado; a fila do message_filter enche e os scans são descartados
# (mapa "malformed"). 2.0 = 2x tempo real, ainda dá pro pipeline acompanhar.
SIM_REAL_TIME_FACTOR = 2.0


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

    # Diretório de instalação do prm_2026 (provedor do mundo + modelos)
    pkg_prm = FindPackageShare("prm_2026").find("prm_2026")
    world_path = PathJoinSubstitution([pkg_prm, "world", LaunchConfiguration("world")])

    def set_physics(context, *args, **kwargs):
        """Aumenta o passo da física via serviço do Ignition (mundo já no ar).

        Lê o nome do <world> dentro do .sdf resolvido para montar o tópico
        /world/<nome>/set_physics e sobe o max_step_size (roda mais rápido).
        """
        world_file = os.path.join(
            pkg_prm, "world", LaunchConfiguration("world").perform(context)
        )
        try:
            with open(world_file, "r") as fh:
                match = re.search(r'<world\s+name="([^"]+)"', fh.read())
            world_name = match.group(1) if match else None
        except OSError:
            world_name = None

        if not world_name:
            return []

        req = "max_step_size: {step}, real_time_factor: {rtf}".format(
            step=SIM_MAX_STEP_SIZE, rtf=SIM_REAL_TIME_FACTOR
        )
        set_physics_proc = ExecuteProcess(
            cmd=[
                "ign",
                "service",
                "-s",
                "/world/{0}/set_physics".format(world_name),
                "--reqtype",
                "ignition.msgs.Physics",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                req,
            ],
            output="screen",
            shell=False,
        )
        # Atraso curto para o Gazebo já ter registrado o serviço do mundo.
        return [TimerAction(period=5.0, actions=[set_physics_proc])]

    gazebo = ExecuteProcess(
        cmd=[
            "ruby",
            FindExecutable(name="ign"),
            "gazebo",
            "-r",
            "-v",
            gz_verbosity,
            world_path,
        ],
        output="screen",
        additional_env=gz_env,
        shell=False,
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
        [
            world_file_arg,
            gz_set_env,
            world_bridge,
            gazebo,
            OpaqueFunction(function=set_physics),
        ]
    )
