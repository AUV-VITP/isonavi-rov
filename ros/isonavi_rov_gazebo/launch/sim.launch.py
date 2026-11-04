import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Remove conflicting NVIDIA GLX offload variables that cause X11 BadValue crash on hybrid graphics
    for var in ("__NV_PRIME_RENDER_OFFLOAD", "__GLX_VENDOR_LIBRARY_NAME"):
        if var in os.environ:
            del os.environ[var]

    # Route EGL rendering to Mesa (Intel integrated graphics) to prevent NVIDIA library mismatch crash in headless/OGRE
    mesa_egl_file = "/usr/share/glvnd/egl_vendor.d/50_mesa.json"
    if os.path.exists(mesa_egl_file):
        os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = mesa_egl_file

    pkg_gazebo = get_package_share_directory("isonavi_rov_gazebo")
    pkg_desc = get_package_share_directory("isonavi_rov_description")

    headless = LaunchConfiguration("headless")
    camera_arg = LaunchConfiguration("camera")
    mode = LaunchConfiguration("mode")
    scattering = LaunchConfiguration("scattering")
    world_name = LaunchConfiguration("world")

    world_path = PathJoinSubstitution([
        pkg_gazebo, "worlds",
        PythonExpression(["'", world_name, "' if '", world_name, "'.endswith('.sdf') else '", world_name, "' + '.sdf'"])
    ])
    bridge_config = os.path.join(pkg_gazebo, "config", "ros_gz_bridge.yaml")
    models_path = os.path.join(pkg_desc, "models")

    # Discover simulation module path for ROS nodes (safety, pilot, vision)
    candidates = [
        os.environ.get("ISONAVI_SIMULATION_PATH", ""),
        os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", "simulation")),
        os.path.abspath(os.path.join(pkg_gazebo, "..", "..", "..", "..", "simulation")),
        os.path.expanduser("~/isonavi/isonavi-rov/simulation"),
        os.path.expanduser("~/isonavi_rov_ws/src/isonavi_rov_gazebo/../../simulation"),
        os.path.join(os.getcwd(), "simulation"),
        os.path.join(os.getcwd(), "isonavi-rov", "simulation"),
    ]
    sim_dir = ""
    for cand in candidates:
        if cand and os.path.isdir(os.path.join(cand, "isonavi")):
            sim_dir = cand
            break

    current_python_path = os.environ.get("PYTHONPATH", "")
    new_python_path = f"{sim_dir}:{current_python_path}" if current_python_path else sim_dir
    set_python_env = SetEnvironmentVariable("PYTHONPATH", new_python_path)

    # Set Gazebo resource path to discover isonavi_rov model
    current_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_resource_path = f"{models_path}:{current_resource_path}" if current_resource_path else models_path

    set_env = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", new_resource_path)
    gz_env = {"GZ_SIM_RESOURCE_PATH": new_resource_path}
    if os.path.exists(mesa_egl_file):
        gz_env["__EGL_VENDOR_LIBRARY_FILENAMES"] = mesa_egl_file

    # Gazebo Harmonic headless vs GUI process
    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "1", world_path],
        additional_env=gz_env,
        output="screen",
        condition=IfCondition(headless),
        on_exit=Shutdown(),
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "1", world_path],
        additional_env=gz_env,
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

    # Dynamic Flood Simulation Controller (water surge, currents, turbidity)
    flood_controller_node = Node(
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

    # Automatic Camera Feed Visualizer Window (Matsya 7 Enhanced Stream + Spatial HUD)
    camera_viewer_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=["/camera/image_detections"],
        output="screen",
        condition=IfCondition(camera_arg),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="large_river", description="Gazebo world: large_river, indian_river, or pool_course"),
        DeclareLaunchArgument("headless", default_value="false", description="Run Gazebo in headless mode (set true for no GUI)"),
        DeclareLaunchArgument("camera", default_value="true", description="Automatically open camera feed viewer window"),
        DeclareLaunchArgument("mode", default_value="assist", description="Pilot mode: manual, assist, auto, shared"),
        DeclareLaunchArgument("scattering", default_value="1.6", description="Water scattering turbidity b (1/m). Murky river: 1.6 - 2.5"),
        set_python_env,
        set_env,
        gz_headless,
        gz_gui,
        bridge_node,
        thruster_bridge_node,
        depth_adapter_node,
        flood_controller_node,
        safety_node,
        pilot_node,
        vision_node,
        camera_viewer_node,
    ])
