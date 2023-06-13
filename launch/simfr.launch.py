import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = 'my_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'view.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    robot_localization_file_path = os.path.join(get_package_share_directory('my_bot'), 'config' , 'ekf_with_gps1.yaml')

    #default_rviz_config_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'nav2_config.rviz')

    #add the word

    world_path = os.path.join(get_package_share_directory('my_bot'), 'worlds' , 'outdoor.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    #gazebo1 = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource([os.path.join(
    #                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #         )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    #spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description',  '-entity', 'my_bot'], output='screen')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-entity', 'my_bot', '-topic', 'robot_description'],
        output='screen'
    )

    ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('my_bot'), 'config' , 'ekf_with_gps.yaml') ]
          )
    
    #virtual gps node
    vitual_gps_node = Node(
            package='my_bot',
            executable='vitual_gps.py',
            name='vitual_gps_node',
            output='screen',
            #parameters=[os.path.join(get_package_share_directory('my_bot'), 'config' , 'ekf_with_gps.yaml') ]
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
    
    # Management virtual gps
    #Start the navsat transform node which converts GPS Vitual data into the world coordinate frame
    start_navsat_transform_cmd_vitual = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_vitual',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('imu', 'imu'),
                    ('gps/fix', 'gps_vitualv'), 
                    ('gps/filtered', 'gps/filteredv'),
                    ('odometry/gps', 'odometry/gpsv'),
                    ('odometry/filtered', 'odometry/globalv')])
    

    # Start robot localization using an Extended Kalman filter...map->odom transform (vitual)
    start_robot_localization_global_cmd_vitual = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map_vitual',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/globalv'),
                    ('/set_pose', '/initialpose')])

    
    



    # Launch them all!
    return LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'),

        #launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                     description='Absolute path to rviz config file'),

        rsp,
        #static_transform,
        #gazebo,
        spawn_entity,
        #ekf,
        start_robot_localization_local_cmd,
        start_robot_localization_global_cmd,
        start_navsat_transform_cmd,
        #start_robot_localization_global_cmd_vitual,
        #start_navsat_transform_cmd_vitual,
        #vitual_gps_node,
        #diff_drive_spawner,
        #joint_broad_spawner,
        #rviz_node
        #gazebo
    ])
