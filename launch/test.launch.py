import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = 'my_bot' #<--- CHANGE ME

    static_map_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'map.yaml')
    nav2_params_path = os.path.join(get_package_share_directory('my_bot'), 'worlds', 'nav2_gps1.yaml')
    default_rviz_config_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'nav2_config.rviz')

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')

    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    robot_localization_file_path = os.path.join(get_package_share_directory('my_bot'), 'config' , 'ekf_with_gps1.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    waypoint_follower_params = {
        'WaypointFollower': {
            'ros__parameters': {
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                #'update_frequency': LaunchConfiguration('update_frequency')
            }
        }
    }

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'view.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    world_path = os.path.join(get_package_share_directory('my_bot'), 'worlds' , 'outdoor.sdf')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-entity', 'my_bot', '-topic', 'robot_description'],
        output='screen',
        parameters=[LaunchConfiguration('use_sim_time')]
    )



    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[LaunchConfiguration('use_sim_time')],
        arguments=['-d', LaunchConfiguration('rvizconfig')])
    


      # Lancer le contrôleur
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{'use_sim_time': True}])

    # Lancer le waypoint follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[waypoint_follower_params]
                                   )
    
    # Start the navsat transform node which converts GPS data into the world coordinate frame
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('imu', 'imu'),
                    ('gps/fix', 'gps/fix'), 
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odometry/global')])

  # Start robot localization using an Extended Kalman filter...map->odom transform
    start_robot_localization_global_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global'),
                    ('/set_pose', '/initialpose')])

  # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
    start_robot_localization_local_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local'),
                    ('/set_pose', '/initialpose')])


   

    # Launch them all!
    return LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                             description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),

        launch.actions.DeclareLaunchArgument(name='map', default_value=static_map_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument(name='params_file', default_value=nav2_params_path,
                                             description='Full path to the ROS2 parameters file to use for all launched nodes'),
        launch.actions.DeclareLaunchArgument(name='autostart', default_value='true',
                                             description='Automatically startup the nav2 stack'),
        launch.actions.DeclareLaunchArgument(name='default_bt_xml_filename', default_value=behavior_tree_xml_path,
                                             description='Full path to the behavior tree xml file to use'),
        launch.actions.DeclareLaunchArgument(name='waypoints_file', default_value='/home/franklin/ws/src/my_bot/worlds/waypoints.yaml',
                                             description='Chemin absolu vers le fichier YAML de waypoints'),

        # Launch the ROS 2 Navigation Stack
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'),
                              #'slam': LaunchConfiguration('slam'),
                              'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'params_file': LaunchConfiguration('params_file'),
                              'default_bt_xml_filename': LaunchConfiguration('default_bt_xml_filename'),
                              'autostart': LaunchConfiguration('autostart')
                              }.items()),

        rsp,
        rviz_node,
        spawn_entity,
        start_navsat_transform_cmd,
        start_robot_localization_global_cmd,
        start_robot_localization_local_cmd,
        #waypoint_follower,
        #controller_server
        
    ])
