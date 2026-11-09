"""Gazebo world, the ROS-Gazebo bridge and the vehicle's nodes.

The pool world and the model are generated (make_world.py, make_model.py). The
tether drag runs inside Gazebo from simulation/isonavi, so that package must be
importable: set ISONAVI_ROOT to the repository, as test_sim.sh does; a symlink
install finds it without.

camera:=true opens the operator video and gcs:=true the ground station; either
one starts the mud and operator view nodes. mesa:=true routes rendering to Mesa,
for hybrid graphics laptops where the NVIDIA libraries crash Gazebo.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    Shutdown,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("isonavi_rov_gazebo")
    pkg_desc = get_package_share_directory("isonavi_rov_description")

    headless = LaunchConfiguration("headless")
    mode = LaunchConfiguration("mode")
    scattering = LaunchConfiguration("scattering")
    world = LaunchConfiguration("world")
    camera = LaunchConfiguration("camera")
    gcs = LaunchConfiguration("gcs")
    mesa = LaunchConfiguration("mesa")
    view = IfCondition(PythonExpression(["'", camera, "' == 'true' or '", gcs, "' == 'true'"]))

    world_path = PathJoinSubstitution([pkg_gazebo, "worlds", [world, ".sdf"]])
    bridge_config = os.path.join(pkg_gazebo, "config", "ros_gz_bridge.yaml")
    models_path = os.path.join(pkg_desc, "models")

    # Set Gazebo resource path to discover isonavi_rov model
    current_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_resource_path = f"{models_path}:{current_resource_path}" if current_resource_path else models_path

    set_env = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", new_resource_path)
    root = os.environ.get("ISONAVI_ROOT", "")
    if not root:
        here = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", ".."))
        root = here if os.path.isdir(os.path.join(here, "simulation", "isonavi")) else ""
    py_path = os.pathsep.join(p for p in (os.path.join(root, "simulation") if root else "",
                                          os.environ.get("PYTHONPATH", "")) if p)
    set_py = SetEnvironmentVariable("PYTHONPATH", py_path)

    use_mesa = [
        UnsetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", condition=IfCondition(mesa)),
        UnsetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", condition=IfCondition(mesa)),
        SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/50_mesa.json",
                               condition=IfCondition(mesa)),
    ]

    # Gazebo Harmonic headless vs GUI process; everything stops when it exits
    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "1", world_path],
        output="screen",
        condition=IfCondition(headless),
        on_exit=Shutdown(),
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "1", world_path],
        output="screen",
        condition=UnlessCondition(headless),
        on_exit=Shutdown(),
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

    # Water current, driven by the flood topic
    flood_node = Node(
        package="isonavi_rov_gazebo",
        executable="flood_controller",
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

    # Video for the pilot; not part of the control loop
    mud_node = Node(
        package="isonavi_rov_gazebo",
        executable="mud_camera",
        output="screen",
        condition=view,
    )
    operator_node = Node(
        package="isonavi_rov_ros",
        executable="operator_view",
        parameters=[{"image_topic": "camera/image_raw_turbid"}],
        output="screen",
        condition=view,
    )
    viewer_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=["/camera/image_detections"],
        output="screen",
        condition=IfCondition(camera),
    )
    ground_station_node = Node(
        package="isonavi_rov_ros",
        executable="ground_station",
        parameters=[{"world": world}],
        output="screen",
        condition=IfCondition(gcs),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="pool_course", description="pool_course, indian_river or large_river"),
        DeclareLaunchArgument("headless", default_value="true", description="Run Gazebo in headless mode"),
        DeclareLaunchArgument("mode", default_value="assist", description="Pilot mode: manual, assist, auto, shared"),
        DeclareLaunchArgument("scattering", default_value="0.4", description="Water scattering turbidity b (1/m)"),
        DeclareLaunchArgument("camera", default_value="false", description="Open the operator video"),
        DeclareLaunchArgument("gcs", default_value="false", description="Open the ground station"),
        DeclareLaunchArgument("mesa", default_value="false", description="Render with Mesa, for hybrid graphics"),
        set_env,
        set_py,
        *use_mesa,
        gz_headless,
        gz_gui,
        bridge_node,
        thruster_bridge_node,
        depth_adapter_node,
        flood_node,
        safety_node,
        pilot_node,
        vision_node,
        mud_node,
        operator_node,
        viewer_node,
        ground_station_node,
    ])
