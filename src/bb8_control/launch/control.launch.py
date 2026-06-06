import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controle_node = Node(
        package="bb8_control",
        executable="controle_robo",
        name="controle_robo",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([controle_node])
