from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value='empty.world',
        description='Nome do arquivo .world do mundo a ser carregado'
    )

    world_file = LaunchConfiguration('world')

    pkg_share = FindPackageShare("prm_2026").find("prm_2026")

    world_path = PathJoinSubstitution([
        pkg_share,
        "world",
        world_file
    ])

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, "models")
    )

    gazebo = ExecuteProcess(
        cmd=[
            FindExecutable(name='gazebo'),
            '--verbose',
            world_path
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_world_arg,
        set_gazebo_model_path,
        gazebo
    ])
