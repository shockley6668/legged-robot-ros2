from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_bringup',
            executable='bridge_node',
            name='leg_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 921600
            }]
        )
    ])
