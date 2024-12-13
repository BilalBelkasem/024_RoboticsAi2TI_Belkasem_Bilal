from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_mediapipe_bilal',
            executable='gesture_detection',
            name='gesture_detection_node',
            output='screen'
        ),
        Node(
            package='lidar_mediapipe_bilal',
            executable='obstacle_avoidance',
            name='obstacle_avoidance_node',
            output='screen'
        )
    ])
