import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Chemin vers le fichier de configuration Cartographer
    config_file = os.path.join(get_package_share_directory('my_bot'), 'worlds', 'my_cartographer1.lua')

    return LaunchDescription([
        # Lancement du nœud Cartographer
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
