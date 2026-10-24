import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("isonavi_rov_gazebo")
    pkg_desc = get_package_share_directory("isonavi_rov_description")

    headless = LaunchConfiguration("headless")
    mode = LaunchConfiguration("mode")
    scattering = LaunchConfiguration("scattering")

    world_path = os.path.join(pkg_gazebo, "worlds", "pool_course.sdf")
    bridge_config = os.path.join(pkg_gazebo, "config", "ros_gz_bridge.yaml")
    models_path = os.path.join(pkg_desc, "models")

    # Set Gazebo resource path to discover isonavi_rov model
    current_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_resource_path = f"{models_path}:{current_resource_path}" if current_resource_path else models_path

    set_env = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", new_resource_path)

    # Gazebo Harmonic headless vs GUI process
    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "1", world_path],
        output="screen",
        condition=IfCondition(headless),
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "1", world_path],
        output="screen",
        condition=UnlessCondition(headless),
    )

    # ROS <-> Gazebo Bridge
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    # Bridge adapter: splits /thrusters array to 8 individual joint thrust topics
    thruster_bridge_node = Node(
        package="isonavi_rov_gazebo",
        executable="thruster_bridge",
        output="screen",
    )

    # Depth adapter: converts hydrostatic pressure / odometry to standard /depth
    depth_adapter_node = Node(
        package="isonavi_rov_gazebo",
        executable="depth_adapter",
        output="screen",
    )

    # isonavi-rov Core Nodes
    safety_node = Node(
        package="isonavi_rov_ros",
        executable="safety",
        output="screen",
    )

    pilot_node = Node(
        package="isonavi_rov_ros",
        executable="pilot",
        parameters=[{"mode": mode, "scattering": scattering}],
        output="screen",
    )

    vision_node = Node(
        package="isonavi_rov_ros",
        executable="vision",
        parameters=[{"scattering": scattering}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true", description="Run Gazebo in headless mode"),
        DeclareLaunchArgument("mode", default_value="assist", description="Pilot mode: manual, assist, auto, shared"),
        DeclareLaunchArgument("scattering", default_value="0.4", description="Water scattering turbidity b (1/m)"),
        set_env,
        gz_headless,
        gz_gui,
        bridge_node,
        thruster_bridge_node,
        depth_adapter_node,
        safety_node,
        pilot_node,
        vision_node,
    ])
