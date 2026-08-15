from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_websocket_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
    )

    web_video_server_node = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
    )

    rosbridge_node = Node(
        package="backend",
        executable="rosbridge_node",
        name="rosbridge_node",
        output="screen",
    )

    thruster_allocator = Node(
        package="auv_control",
        executable="thruster_allocator",
        name="thruster_allocator",
        output="screen",
    )
    return LaunchDescription(
        [
            rosbridge_websocket_node,
            web_video_server_node,
            rosbridge_node,
            thruster_allocator,
        ]
    )
