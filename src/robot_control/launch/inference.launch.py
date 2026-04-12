import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='inference_node',
            name='inference_node',
            output='screen',
            parameters=[
                {
                    'model_path_0406': '/root/legged-robot/src/robot_control/robot_control/model_0406.onnx',
                    'model_path_0407': '/root/legged-robot/src/robot_control/robot_control/model_0407.onnx'
                }
            ]
        )
    ])
