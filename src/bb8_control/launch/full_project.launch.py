import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_control = FindPackageShare("bb8_control")

    # Launch parcial (Simulação + Carregamento do Robô + SLAM)
    inclui_infraestrutura = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_control.find("bb8_control"), "launch", "simulacao_slam.launch.py"
            )
        )
    )

    # Launch específico de controle (que ativa o nó controle_robo)
    inclui_controle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_control.find("bb8_control"), "launch", "controle.launch.py"
            )
        )
    )

    return LaunchDescription([inclui_infraestrutura, inclui_controle])
