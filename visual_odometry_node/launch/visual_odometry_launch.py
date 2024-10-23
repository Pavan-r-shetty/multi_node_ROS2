from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='visual_odometry_node',
            executable='visual_odometry_node',
            name='visual_odometry_node',
            output='screen',
            parameters=[
                {'config_file': '/home/shetty/ros2_ws/src/ORB_SLAM3/Examples/RGB-D/custom_camera.yaml'}  # Adjust the path to your config file
            ]
        )
    ])
