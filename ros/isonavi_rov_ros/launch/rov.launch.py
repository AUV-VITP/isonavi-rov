from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    port = LaunchConfiguration("port")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="assist"),
        DeclareLaunchArgument("port", default_value=""),
        Node(package="joy", executable="joy_node", name="joy"),
        Node(package="isonavi_rov_ros", executable="pilot", parameters=[{"mode": mode}]),
        Node(package="isonavi_rov_ros", executable="safety"),
        Node(package="isonavi_rov_ros", executable="actuator", parameters=[{"port": port}]),
        Node(package="isonavi_rov_ros", executable="vision"),
    ])
