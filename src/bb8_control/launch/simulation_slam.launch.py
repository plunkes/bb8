import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Pacotes dependentes
    pkg_simulacao = FindPackageShare("prm_2026")
    pkg_slam = FindPackageShare("bb8_slam")

    # Inicia o mundo no Gazebo
    inclui_simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_simulacao.find("prm_2026"), "launch", "inicia_simulacao.launch.py"
            )
        )
    )

    # Carrega os controladores do robô e o robot_state_publisher
    inclui_carrega_robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_simulacao.find("prm_2026"), "launch", "carrega_robo.launch.py"
            )
        )
    )

    # Ativa o SLAM Toolbox
    inclui_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam.find("bb8_slam"), "launch", "slam_launch.py")
        )
    )

    # Retorna a descrição contendo toda a infraestrutura física e sensorial unificada
    return LaunchDescription([inclui_simulacao, inclui_carrega_robo, inclui_slam])
