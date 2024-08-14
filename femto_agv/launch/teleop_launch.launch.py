from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Include the launch_robot.launch.py file from the femto_agv package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('femto_agv'),
                    'launch',
                    'launch_robot.launch.py'
                ])
            ])
        ),

        # Node for teleop_twist_keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            parameters=[
                {'stamped': True}
            ],
            remappings=[
                ('/cmd_vel', '/diff_cont/cmd_vel')
            ]
        )
    ])

