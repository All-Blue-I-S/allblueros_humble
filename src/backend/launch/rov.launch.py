import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mavros_dir = get_package_share_directory("mavros")
    backend_dir = get_package_share_directory("backend")
    oak_d_dir = get_package_share_directory("oak_d")

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mavros_dir, "launch", "apm.launch.py")
        ),
        launch_arguments={"fcu_url": "/dev/ttyACM0:115200"}.items(),
    )

    trigger_node = Node(
        package="backend",
        executable="trigger_service",
        name="service_trigger",
        output="screen",
    )

    hud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(backend_dir, "launch", "hud.launch.py")
        )
    )

    oak_d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(oak_d_dir, "launch", "camera.launch.py")
        )
    )
    return LaunchDescription([mavros_launch, trigger_node, hud_launch, oak_d_launch])
