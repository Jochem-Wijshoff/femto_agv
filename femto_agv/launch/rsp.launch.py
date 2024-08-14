import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Check if we're told to use sim time or ros2_control
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')

    # Process the URDF file using xacro
    pkg_path = os.path.join(get_package_share_directory('femto_agv'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    
    # Command to process xacro file
    robot_description_command = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time
    ])
    
    # Create a robot_state_publisher node with parameters
    params = {
        'robot_description': robot_description_command,
        'use_sim_time': use_sim_time
    }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Return the LaunchDescription object
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        ),
        node_robot_state_publisher
    ])
