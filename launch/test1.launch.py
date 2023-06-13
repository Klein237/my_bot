import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

def generate_launch_description():
    # Chemin absolu du fichier de configuration
    #params_file_path = os.path.join(os.getcwd(), 'worlds', 'nav2_gps2.yaml')
    nav2_params_path = os.path.join(get_package_share_directory('my_bot'), 'worlds', 'nav2_gps2.yaml')
    


    # Nodes
    gps_node = launch_ros.actions.Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver_node',
        name='nmea_serial_driver',
        parameters=[{'port': '/dev/ttyACM0', 'baudrate': '9600'}]
    )

    nav2_node = launch_ros.actions.Node(
        package='nav2_bringup',
        executable='bringup',
        name='nav2_bringup',
        parameters=[nav2_params_path]
    )

    waypoints_publisher_node = launch_ros.actions.Node(
        package='rosbag',
        executable='play',
        name='waypoints_publisher',
        parameters=[
            {'hz': '10', 'pause': 'true', 'queue': '50', 'start_paused': 'true',
             'clock': '/clock', 'topics': '/waypoints:=/path',
             'bags': '/path/to/waypoints.bag'}
        ]
    )

    ekf_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_localization_node',
        name='ekf_localization_node',
        parameters=[
            {'use_sim_time': 'true', 'publish_tf': 'true', 'output_frame': 'map',
             'base_link_frame': 'base_link', 'world_frame': 'map',
             'sensor_timeout': '0.1', 'two_d_mode': 'false'}
        ],
        remappings=[('gps/fix', '/odometry/gps')]
    )
# Launch the ROS 2 Navigation Stack
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart')
        }.items()
    )

    # Launch description
    ld = launch.LaunchDescription()

    # Add nodes to the launch description
    #ld.add_action(gps_node)
    ld.add_action(nav2_node)
    #ld.add_action(waypoints_publisher_node)
    ld.add_action(ekf_localization_node)
    ld.add_action(bringup_launch)

    return ld




