import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'femto_agv'  # Adjust this to your package name
    lidar_package = 'sllidar_ros2'

    package_share_directory = get_package_share_directory(package_name)
    lidar_share_directory = get_package_share_directory(lidar_package)

    rviz_config_dir = package_share_directory

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_share_directory, 'launch', 'rsp.launch.py')]
        ), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    sllidar_c1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(lidar_share_directory, 'launch', 'sllidar_c1_launch.py')]
        ),
    )
    
    localization_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_share_directory, 'launch', 'localization_launch.py')]
        ), launch_arguments={'map': os.path.join(package_share_directory, 'maps', 'pid_map.yaml')}.items()
    )
    
    navigation_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_share_directory, 'launch', 'navigation_launch.py')]
        ), launch_arguments={'map_subscribe_transient_local': 'true'}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(package_share_directory, 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['diff_cont'],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_broad'],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    joy_params = os.path.join(package_share_directory, 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/cmd_vel_joy')]
    )

    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        parameters=[{'frame_id': 'base_link'}],
        remappings=[
            ('cmd_vel_in', '/diff_cont/cmd_vel_mux'),
            ('cmd_vel_out', '/diff_cont/cmd_vel')
        ]
    )

    twist_mux_params_file = os.path.join(package_share_directory, 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params_file],
        remappings=[
            ('cmd_vel_out', 'diff_cont/cmd_vel_mux')
        ]
    )

    rviz_config = os.path.join(rviz_config_dir, 'config', 'rviz_config_file.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        sllidar_c1_launch,
        joy_node,
        teleop_node,
        twist_stamper_node,
        twist_mux_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        rviz_node,
        localization_nav2,
        navigation_nav2,
    ])
