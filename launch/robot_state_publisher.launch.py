"""robot_state_publisher do BB8.

Expande o Xacro LOCAL do bb8_control (description/robot.urdf.xacro) e publica
o robot_description + os TFs estáticos dos links. O Xacro foi migrado do
prm_2026 para cá, de modo que o bb8_control é autocontido quanto à descrição
do robô (braço, câmeras, LIDAR, juntas de controle).
"""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("bb8_control"), "description", "robot.urdf.xacro"]
    )
    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([robot_state_publisher_node])
