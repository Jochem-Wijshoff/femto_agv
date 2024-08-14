from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    try:
        joy_params = os.path.join(get_package_share_directory('femto_agv'), 'config', 'joystick.yaml')
        
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
            remappings=[('/cmd_vel', '/diff_cont/cmd_vel')]
        )

        return LaunchDescription([
            joy_node,
            teleop_node,
        ])
    
    except Exception as e:
        raise RuntimeError(f"Failed to generate launch description: {e}")

if __name__ == '__main__':
    # The presence of this block allows executing the script directly for quick testing.
    import launch
    launch_service = launch.LaunchService()
    description = generate_launch_description()
    launch_service.include_launch_description(description)
    launch_service.run()