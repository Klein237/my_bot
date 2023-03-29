import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

#import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    urdf = os.path.join(pkg_path,'description','rover.urdf')
    #robot_description_config = xacro.process_file(xacro_file)
    #robot_description_config = xacro.process_file(xacro_file).toxml()
    #robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control])
    
    # Create a robot_state_publisher node
    params = { 'use_sim_time': use_sim_time}
   # params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[urdf],
        parameters=[params],
        remappings=remappings
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        arguments=[urdf]
       

    )



    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        
       

        node_robot_state_publisher,
        joint_state_publisher_node
    ])
