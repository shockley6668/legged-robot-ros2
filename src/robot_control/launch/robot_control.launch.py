import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    joy_to_cmd_vel_node = Node(
        package='robot_control',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        parameters=[{
            'axis_linear_x': 1,
            'axis_linear_y': 0,
            'axis_angular_z': 3,
            'scale_linear': 0.5,
            'scale_angular': 1.45,
        }]
    )

    inference_node = Node(
        package='robot_control',
        executable='inference_node',
        name='inference_node',
        output='screen',
        parameters=[{
            'model_path': "/root/legged-robot/src/robot_control/robot_control/finall.onnx"
        }]
    )

    return LaunchDescription([
        joy_node,
        joy_to_cmd_vel_node,
        inference_node
    ])
