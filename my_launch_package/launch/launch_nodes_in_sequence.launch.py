from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, EmitEvent
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    # Define the second node (visual_odometry_node) to start first
    visual_odometry_node = Node(
        package='visual_odometry_node',  # Replace with the actual package name
        executable='visual_odometry_node',
        name='visual_odometry_node'
    )

    # Define the first node (image_loader_node) to start after a 3-second delay
    image_loader_node_node = Node(
        package='image_loader_node',  # Replace with the actual package name
        executable='image_loader_node',
        name='image_loader_node',
        parameters=[{
            'left_image_dir': '/home/shetty/ros2_ws/rainier_labs/assignmentData/left_0/',
            'depth_image_dir': '/home/shetty/ros2_ws/rainier_labs/assignmentData/depth_0/'
        }]
    )
    image_loader_node = TimerAction(
        period=3.0,
        actions=[image_loader_node_node]
    )

    # Define the third node (point_cloud_stitcher_node) to start after a delay
    point_cloud_stitcher_node = TimerAction(
        period=4.0,  # Starts 4 seconds after launch begins
        actions=[Node(
            package='point_cloud_stitcher_node',  # Replace with the actual package name
            executable='point_cloud_stitcher_node',
            name='point_cloud_stitcher_node'
        )]
    )

    # Event handler to trigger a 60-second timer after image_loader_node ends
    exit_after_60s = RegisterEventHandler(
        OnProcessExit(
            target_action=image_loader_node_node,  # Use the Node action here
            on_exit=[TimerAction(
                period=60.0,  # Wait for 60 seconds after image_loader_node_node exits
                actions=[EmitEvent(event=Shutdown())]
            )]
        )
    )

    return LaunchDescription([
        visual_odometry_node,
        image_loader_node,
        point_cloud_stitcher_node,
        exit_after_60s
    ])
