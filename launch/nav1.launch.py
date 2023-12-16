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

    static_map_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'my_map1.yaml')
    nav2_params_path = os.path.join(get_package_share_directory('my_bot'), 'worlds', 'nav2_params5.yaml')
    default_rviz_config_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'nav2_config.rviz')

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')

    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    #add the word

    world_path = os.path.join(get_package_share_directory('my_bot'), 'worlds' , 'wordtest.sdf')

    

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    #spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description',  '-entity', 'my_bot'], output='screen')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-entity', 'my_bot', '-topic', 'robot_description'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[LaunchConfiguration('use_sim_time')],
        arguments=['-d', LaunchConfiguration('rvizconfig')])

    static_transform = Node(package="tf2_ros",
                            executable="static_transform_publisher",
                            # arguments = ["-5.0", "-30.0", "0.", "0.0", "0", "0.0", "map", "odom"])
                            arguments=["0.0", "0.0", "0.", "0.0", "0", "0.0", "map", "odom"])

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
        launch.actions.DeclareLaunchArgument(name='slam', default_value='False',
                                             description='Whether to run SLAM'),

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
        #static_transform,
        spawn_entity,
        #diff_drive_spawner,
        #joint_broad_spawner,
        
    ])
