from launch import LaunchDescription
import launch_ros
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '4/camera_link', 'camera_link']
        ),

        Node(
            package='environment_model',
            executable='environment_model',
            name='environment_model',
            output='screen',
            parameters=[{
            }]
        ),


    ])
