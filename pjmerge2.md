# ISONAVI-ROV: Autonomous & Teleoperated Underwater Inspection Vehicle
## Technical Architecture, 3D Simulation Twin, Vision Enhancement & Operations Manual (`pjmerge2.md`)

**ROS 2 Jazzy Jalisco + Gazebo Harmonic (gz-sim 8.15.0) on Ubuntu 24.04.5 LTS (Noble Numbat)**

---

## 1. Executive Summary & System Overview

This document provides a comprehensive, production-grade technical manual for the **ISONAVI-ROV** project. It chronicles the full architecture, hydrodynamics, optical image restoration, autonomous dynamic positioning, safety gating, and simulation infrastructure developed for the 8-thruster vectored underwater inspection robot.

The system is designed for freshwater inspection, flood response, and competitive underwater robotics (inspired by state-of-the-art platforms such as **IIT Bombay Matsya 6 / Matsya 7**, SAUVC, and RoboSub).

### Key Vehicle Specifications

| Parameter | Specification | Physical Context & Significance |
| :--- | :--- | :--- |
| **Dry Mass ($m$)** | $10.490\,\text{kg}$ | Calibrated structural frame, electronics enclosure, and payload |
| **Displacement Volume ($V$)** | $10.700\,\text{L}$ ($0.0107\,\text{m}^3$) | Calibrated displacement of syntactic buoyancy foam and chassis |
| **Net Buoyancy ($F_B - F_g$)** | $+2.00\,\text{N}$ (Net positive) | Vehicle naturally floats to surface at $\approx 0.15\,\text{m/s}$ in failsafe / blackout |
| **Center of Gravity ($\mathbf{r}_g$)** | $[0.000, 0.000, -0.0410]\,\text{m}$ | Low-slung battery and electronics ballast |
| **Center of Buoyancy ($\mathbf{r}_b$)** | $[0.000, 0.000, +0.0040]\,\text{m}$ | Upper syntactic foam placement |
| **Metacentric Height ($\overline{GM} / BG$)** | $45.0\,\text{mm}$ | Passive righting moment: self-rights against roll/pitch perturbations |
| **Thruster Topology** | 8 Vectored (4 Horiz $\pm 45^\circ$, 4 Vert) | Full holonomic 6-DoF control: independent surge, sway, heave, yaw |
| **Propulsion Type** | 8 $\times$ T200/Custom $76\,\text{mm}$ Ducted | Peak thrust $\pm 52\,\text{N}$ per thruster; nominal operational limit $\pm 28\,\text{N}$ |
| **Optical Suite** | $1280 \times 720$ RGB @ $30\,\text{Hz}$, $80^\circ$ FOV | Forward-facing inspection camera backed by dual $1200\,\text{lm}$ spotlights |
| **Inertial & Depth Sensors** | 6-Axis IMU ($100\,\text{Hz}$) + Pressure ($50\,\text{Hz}$) | Low-noise linear acceleration, angular velocity, and millimeter depth tracking |

---

## 2. System Architecture & Component Interaction

```mermaid
flowchart TD
    subgraph Human_Interface["Pilot & Supervisory Control"]
        Teleop["Keyboard Teleop / Joy\n(keyboard_teleop.py)"]
        FloodCTL["Flood Controller CLI\n(flood_ctl.py)"]
        RqtViewer["Matsya 7 Camera HUD\n(rqt_image_view)"]
        RqtGraph["ROS 2 Graph Viewer\n(rqt_graph)"]
    end

    subgraph ROS2_Core["ROS 2 Jazzy Core Stack (isonavi_rov_ros)"]
        PilotNode["Pilot Flight Controller\n(pilot_node.py)\n- Active DP & Station Hold\n- Damped PID Heave/Yaw\n- Output Low-Pass Filter"]
        SafetyNode["Safety Supervisor\n(safety_node.py)\n- Heartbeat Watchdog\n- Tilt / Depth Gating\n- Failsafe State Machine"]
        VisionNode["Advanced Optical AI\n(vision_node.py)\n- Pyramidal Retinex (Matsya 7)\n- CLAHE Color Balance\n- YOLOv8 Object Detection"]
    end

    subgraph Simulation_Bridge["Gazebo <-> ROS 2 Bridge (isonavi_rov_gazebo)"]
        RosGzBridge["ros_gz_bridge\n(parameter_bridge)"]
        ThrusterBridge["Thruster Demux Bridge\n(thruster_bridge.py)"]
        DepthAdapter["Depth Sensor Adapter\n(depth_adapter.py)"]
        FloodSim["Dynamic Flood Controller\n(flood_controller.py)"]
    end

    subgraph Gazebo_Harmonic["Gazebo Harmonic Engine (gz-sim 8.15.0)"]
        Physics["DART Physics Engine\n- Implicit Fluid Added Mass\n- 6-DoF Equations of Motion"]
        Hydro["gz-sim-hydrodynamics-system\n- Fossen Nonlinear Damping\n- River Flow Vector"]
        Buoyancy["gz-sim-buoyancy-system\n- Graded Buoyancy (Water/Air)"]
        ThrustersGZ["8x gz-sim-thruster-system\n- Propeller Joint Dynamics"]
        SensorsGZ["Sensors Engine\n- RGB Camera + Dual Spotlights\n- 6-Axis IMU & Fluid Pressure"]
    end

    Teleop -->|"/joy (sensor_msgs/Joy)"| PilotNode
    FloodCTL -->|"/flood (std_msgs/String)"| FloodSim
    PilotNode -->|"/wrench (geometry_msgs/Wrench)"| SafetyNode
    SafetyNode -->|"/thrusters (Float64MultiArray)"| ThrusterBridge
    ThrusterBridge -->|"Joint cmd_thrust (Float64)"| RosGzBridge
    RosGzBridge --> ThrustersGZ
    FloodSim -->|"Current & Turbidity"| Hydro

    SensorsGZ --> RosGzBridge
    RosGzBridge -->|"/model/isonavi_rov/odometry"| DepthAdapter
    RosGzBridge -->|"/model/isonavi_rov/camera/image"| VisionNode
    RosGzBridge -->|"/imu"| PilotNode
    DepthAdapter -->|"/depth"| PilotNode
    DepthAdapter -->|"/depth"| SafetyNode
    VisionNode -->|"/camera/image_detections"| RqtViewer
```

---

## 3. Host Environment, Driver Safety & Compositor Configuration

### 3.1 Host Hardware & Kernel Specs
* **Operating System:** Ubuntu 24.04.5 LTS (Noble Numbat), 64-bit x86_64 (`Linux 6.8.0-48-generic`)
* **Primary Processor & Integrated Graphics:** Intel Core 13th/14th Gen (Raptor Lake) UHD Graphics (`Mesa 25.2.8`, OpenGL 4.6 Core Profile)
* **Discrete GPU:** NVIDIA GeForce RTX 4050 Max-Q / Mobile
* **Display Server & Compositor:** X11 (`DISPLAY=:1`), running direct hardware acceleration on Intel Mesa

### 3.2 Hardware & GPU Safety Audit (Kernel vs Driver Resolution)
During system inspection, `nvidia-smi` reported:
`Failed to initialize NVML: Driver/library version mismatch`
* **Root Cause:** Ubuntu's automatic background security updates (`unattended-upgrades`) downloaded and installed the `595.99` NVIDIA driver packages at `00:54–00:56`, while the currently loaded Linux kernel module in memory remained on `595.91`.
* **Hardware Safety Status:** The hardware and GPU silicon are completely uncompromised. The condition is standard on Linux workstations following kernel updates and resolves completely on the next routine reboot.
* **Compositor Conflict Resolved:** The conflicting environment variables `__NV_PRIME_RENDER_OFFLOAD=1` and `__GLX_VENDOR_LIBRARY_NAME=nvidia` were removed from [`~/.bashrc`](file:///home/prjwaal/.bashrc). On Intel-driven X11 laptop displays, forcing NVIDIA GLX caused Qt5/X11 to throw `BadValue (integer parameter out of range)` fatal errors, crashing GUI windows.
* **EGL Mesa Routing:** Headless OGRE rendering in Gazebo Harmonic is configured to route via Mesa (`/usr/share/glvnd/egl_vendor.d/50_mesa.json`), preventing any crash or dependency on the un-rebooted NVIDIA module.

### 3.3 Convenient Bash Aliases
The following aliases are installed in [`~/.bashrc`](file:///home/prjwaal/.bashrc):
```bash
# Source ROS 2 Jazzy and ISONAVI workspace in one word:
alias ws-isonavi="source /opt/ros/jazzy/setup.bash && source /home/prjwaal/isonavi_rov_ws/install/setup.bash"

# Cleanly terminate all Gazebo Harmonic and ROS nodes:
alias kill-sim="pkill -9 -f 'gz sim|parameter_bridge|thruster_bridge|depth_adapter|flood_controller|safety|pilot|vision|rqt_image_view' 2>/dev/null || true"

# Launch the camera visualizer:
alias rqt_image_view="ros2 run rqt_image_view rqt_image_view"
```

---

## 4. Deep-Dive Engineering & Problem Resolution

### 4.1 Resolution of Vehicle Vibration in Teleoperation
* **Symptom:** When running `keyboard_teleop.py` or sending `/joy` commands, the vehicle violently shook and oscillated along the heave and roll axes, causing severe water disturbances.
* **Root Causes Diagnosed:**
  1. *Excessive Assist Gains on a Light Vehicle:* `pilot_node.py` previously employed heave gains $k_z = 260.0$, $k_{vz} = 120.0$, and attitude gains $k_{att} = 40.0$, $k_{att,d} = 12.0$. On a $10.49\,\text{kg}$ vehicle, minute discrete depth variations ($1-2\,\text{mm}$) caused heave forces to swing by over $\pm 14\,\text{N}$ in $20\,\text{ms}$.
  2. *Attitude Conflict with Hydrostatic Righting Moment:* With $BG = 45\,\text{mm}$, the vehicle has a strong natural righting moment. Active attitude control gains $> 10\,\text{Nm/rad}$ fought this natural restoring torque, creating mechanical resonance with IMU gyro noise.
  3. *Setpoint Dragging:* When holding down teleoperation keys, the station-keeping setpoint lagged behind the moving vehicle. When keys were released, the controller violently snapped the vehicle backward.
  4. *Unfiltered Discrete Motor Commands:* Step updates from $50\,\text{Hz}$ loops injected high-frequency harmonics directly into the simulated thruster joints.
* **Implemented Solution in [`pilot_node.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_ros/isonavi_rov_ros/pilot_node.py):**
  * Softened and critically damped the PID gains:
    ```python
    kz = 75.0        # Smooth heave tracking (N/m)
    kvz = 35.0       # Velocity damping (N*s/m)
    kyaw = 4.0       # Damped yaw turning (Nm/rad)
    katt = 8.0       # Gentle roll/pitch righting (Nm/rad)
    katt_d = 2.5     # Angular rate damping (Nm*s/rad)
    ```
  * Added dynamic station-keeping decoupling: while joystick inputs are non-zero, setpoints continuously track vehicle state (`self.x_ref = self.x; self.y_ref = self.y`). When keys are released, the vehicle smoothly transitions to holding its final position.
  * Introduced a $2\,\text{cm}$ station-keeping deadband to ignore sensor noise.
  * Added a $50\,\text{Hz}$ low-pass exponential moving average (EMA) filter on the 6-DoF output wrench:
    $$\boldsymbol{\tau}_{\text{smooth}}(k) = 0.65 \cdot \boldsymbol{\tau}_{\text{smooth}}(k-1) + 0.35 \cdot \boldsymbol{\tau}_{\text{cmd}}(k)$$

### 4.2 Resolution of Camera Feed Visualizer Launch Failure
* **Symptom:** Camera visualizer failed to open; running `rqt_image_view` from terminal returned `command not found`.
* **Root Causes Diagnosed:**
  1. In ROS 2 Jazzy, `rqt_image_view` is packaged under `/opt/ros/jazzy/lib/rqt_image_view/rqt_image_view` and is not placed in the system `/usr/bin/` PATH.
  2. The simulation launch file (`sim.launch.py`) did not launch the camera GUI node.
* **Implemented Solution:**
  * Embedded `camera_viewer_node` directly into [`sim.launch.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/launch/sim.launch.py) defaulting to `camera:=true`.
  * Pre-configured the viewer argument to automatically hook into `/camera/image_detections` (Matsya 7 Spatial AI HUD).
  * Provided the `rqt_image_view` bash alias.

### 4.3 Resolution of DART Added Mass Numerical Explosions
* **Symptom:** `gz sim` crashed after 20–50 steps with ODE bounding box overflow (`aabbBound >= dMinIntExact && aabbBound < dMaxIntExact`).
* **Root Cause:** Explicit added mass computation in `gz-sim-hydrodynamics-system` computes $\mathbf{F}_{am} = -\mathbf{M}_A \frac{\mathbf{v}_k - \mathbf{v}_{k-1}}{\Delta t}$. Because heave added mass ($14.71\,\text{kg}$) exceeds dry mass ($10.49\,\text{kg}$), the explicit loop is unconditionally unstable.
* **Implemented Solution in [`model.sdf`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_description/models/isonavi_rov/model.sdf):**
  * Configured SDFormat 1.8 native `<fluid_added_mass>` matrix inside `<link name="base_link"><inertial>`. DART handles added mass implicitly on the left-hand side of the dynamics equations:
    $$(\mathbf{M} + \mathbf{M}_A) \ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{D}(\dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}$$
  * Added `<disable_added_mass>true</disable_added_mass>` to `gz-sim-hydrodynamics-system` to prevent duplicate explicit force calculations.

### 4.4 Waterline Stability via Graded Buoyancy
* **Symptom:** Under uniform fluid density, breaching the water caused the vehicle to accelerate upward into the sky.
* **Implemented Solution in [`worlds/pool_course.sdf`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/worlds/pool_course.sdf) & River Worlds:**
  * Configured `<graded_buoyancy>` with water density ($1000.0\,\text{kg/m}^3$) below $z = 0.0\,\text{m}$ and air density ($1.2\,\text{kg/m}^3$) above $z = 0.0\,\text{m}$.
  * The vehicle now naturally bobs, stabilizes, and rests partially submerged at the surface.

---

## 5. Matsya 6 & 7 Inspired Underwater Vision & AI Pipeline

Underwater imagery in turbid rivers and pools suffers from:
1. **Wavelength-Dependent Light Attenuation:** Red light absorbs within $1-2\,\text{m}$, creating a dark cyan/blue/green cast.
2. **Forward & Backward Light Scattering:** Suspended particulates cause loss of high-frequency edge details and low dynamic contrast.

The vision system in [`vision_node.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_ros/isonavi_rov_ros/vision_node.py) replicates the image processing methodologies employed by **IIT Bombay Matsya 6 and Matsya 7**:

```mermaid
flowchart LR
    Raw["Raw Image Stream\n(/camera/image_raw)"] --> Retinex["Pyramidal Retinex\n(Illumination Invariance)"]
    Retinex --> CLAHE["Lab Space CLAHE\n(Local Contrast Gain)"]
    CLAHE --> ColorRestore["Underwater White Balance\n(Red Channel Compensator)"]
    ColorRestore --> YOLO["YOLOv8 Object Detection\n(Pipes, Gates, Buoys, Valves)"]
    ColorRestore --> SpatialHUD["Spatial Telemetry Overlay\n(Depth, Heading, Horizon)"]
    YOLO --> SpatialHUD
    SpatialHUD --> Output["Enhanced Stream\n(/camera/image_detections)"]
```

### 5.1 Enhancement Algorithms in `vision_node.py`
1. **Multi-Scale Retinex (MSR) with Dynamic Range Compression:**
   Decomposes the image $I(x,y)$ into reflectance $R(x,y)$ and illumination $L(x,y)$:
   $$R_i(x,y) = \sum_{k=1}^K w_k \left( \ln I_i(x,y) - \ln(G_k(x,y) * I_i(x,y)) \right)$$
   using Gaussian scales $\sigma \in \{15, 80, 250\}$ to strip illumination gradients and backscatter fog.
2. **Contrast Limited Adaptive Histogram Equalization (CLAHE):**
   Converts the image to CIE $L^*a^*b^*$ color space, applies CLAHE ($2.5$ clip limit, $8 \times 8$ tile grid) exclusively to the $L^*$ channel to preserve natural color ratios, and re-maps to RGB.
3. **Underwater Red Channel Restoration:**
   Compensates the attenuated red spectrum by redistributing energy from the green and blue channels based on water turbidity.
4. **YOLOv8 Real-Time Neural Object Detection:**
   Executes ultralytics YOLOv8 inference, identifying inspection assets (valves, submerged gates, obstacles, buoys, mooring chains).
5. **Spatial Telemetry HUD Overlay:**
   Overlays an aviation-grade artificial horizon, depth gauge, compass heading tape, thruster arming state, and target bounding boxes with distance estimations.

---

## 6. Dynamic Flood & River Current Simulation

The system provides realistic simulation of dynamic river currents, flash floods, and debris flow through two dedicated components:

### 6.1 `flood_controller.py`
* Node located in [`ros/isonavi_rov_gazebo/isonavi_rov_gazebo/flood_controller.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/isonavi_rov_gazebo/flood_controller.py).
* Simulates fluid surge velocities:
  $$\mathbf{v}_{\text{water}}(t) = \mathbf{v}_{\text{mean}} + \mathbf{A}_{\text{gust}} \sin(\omega t) + \boldsymbol{\eta}_{\text{turbulent}}(t)$$
* Calculates cross-flow hydrodynamic drag forces and moments on the vehicle hull and publishes dynamic parameters to the ROS ecosystem.

### 6.2 `flood_ctl.py` Command-Line Tool
Enables operators to dynamically change river conditions without restarting Gazebo:

```bash
# 1. Switch to calm conditions (0.2 m/s flow, low turbidity)
ros2 run isonavi_rov_gazebo flood_ctl --preset calm

# 2. Switch to moderate flood (0.8 m/s flow, medium turbidity)
ros2 run isonavi_rov_gazebo flood_ctl --preset moderate

# 3. Switch to extreme flash flood (2.0 m/s flow, high debris, heavy turbidity)
ros2 run isonavi_rov_gazebo flood_ctl --preset extreme

# 4. Custom flow, turbidity, and debris count
ros2 run isonavi_rov_gazebo flood_ctl --flow 1.5 --turbidity 2.2 --debris 12
```

### 6.3 Uneven Riverbed Bathymetry, Rock Formations & Debris Collision Physics
* **Expanded & Uneven Riverbed:** Extended the river world [`large_river.sdf`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/worlds/large_river.sdf) to $130\,\text{m} \times 48\,\text{m}$ with non-uniform bathymetry, including upstream sand shoals, central scour dunes, side bank terraces, and deep scour pits ($1.6\,\text{m}$ to $5.8\,\text{m}$ depth).
* **Rock & Boulder Formations:** Added jagged rock outcrops, granite boulder deflector fields around bridge piers, submerged rock shelves, and bedrock spires with full ODE `<collision>` physics.
* **Reduced Murky Dust/Fog & Natural Optical Clarity:**
  - Reduced dense underwater fog density from $0.55$ down to $0.10$, shifted fog `start` from $0.4\,\text{m}$ to $1.5\,\text{m}$, and extended `end` to $16.0\,\text{m}$.
  - Calibrated natural ambient lighting ($0.35, 0.36, 0.32$) and sunlight diffuse levels ($0.62, 0.62, 0.55$), eliminating the overwhelming murky haze while preserving realistic depth attenuation. All debris, riverbed silt, and obstacles are now clearly visible through the ROV forward camera.
* **Small Spherical Dirt Particles with Physical Collisions (Inside Water Only):**
  - **High-Throughput Continuous Drift Emitter:** Integrated underwater box particle emitter (`underwater_spherical_dust_emitter`, rate: $350\,\text{Hz}$, lifetime: $20\,\text{s}$, size: $100\,\text{m} \times 36\,\text{m} \times 2.2\,\text{m}$) spanning billions of drifting micro-dirt specks across operational lifespans strictly below the waterline ($z \in [-3.6, -0.25]\,\text{m}$). Zero dust exists above water.
  - **Discrete Collision Dirt Spheres & Silt Pebbles:** Added physical `<collision>` spherical dirt clusters (`dust_sphere_cluster_1` through `dust_sphere_cluster_6`) and an upstream riverbed silt pebble field (`dirt_pebble_field_upstream`), providing physical collision geometry ($r \in [15, 45]\,\text{mm}$) that physically interacts with the ROV chassis and thruster wash.
* **Comprehensive Plastic Bags Collection (with Collision Physics):**
  - Submerged polythene grocery bags in assorted colors (`submerged_plastic_bag_white_4`, `submerged_plastic_bag_red`, `submerged_plastic_bag_blue`, `submerged_plastic_bag_green`, `submerged_plastic_bag_yellow`).
  - Floating heavy-duty garbage bag (`floating_garbage_bag_2`) and submerged transparent plastic packaging sheet (`submerged_plastic_packaging_sheet`), each configured with rigid `<collision>` boxes.
* **Plastic Bottles & Household Containers (with Collision Physics):**
  - Submerged clear mineral water bottles (`plastic_water_bottle_clear_1`, `2`, `3`, `4`).
  - Submerged green PET soda bottles (`plastic_soda_bottle_green_1`, `2`) and crushed soda bottle (`plastic_soda_bottle_crushed`).
  - High-density polyethylene (HDPE) industrial jugs: orange detergent jug (`plastic_detergent_bottle_orange`), white bleach container (`plastic_bleach_bottle_white`), and blue motor oil jug (`plastic_oil_bottle_blue`).
  - 200L polyethylene chemical drums (floating and submerged), 20L oil jerrycans, rigid industrial plastic crates, discarded plastic utility buckets, wash basins, PVC sewer pipe segments, traffic cones, and automotive bumper panels.
* **Rotten Wooden Items & Driftwood Particles (with Collision Physics):**
  - Rotting driftwood tree trunks with branch stubs (`rotten_driftwood_trunk_1`, `rotten_driftwood_log_2`), gnarled tree root stumps (`gnarled_tree_root_stump`), broken wooden shipping pallets, railway sleepers, and rotting timber beams.
  - Decaying submerged driftwood branches (`wood_particle_branch_1`, `2`, `3`) with collision cylinders.
  - Floating and submerged rotting wood chip clusters (`wood_chip_cluster_1`, `wood_chip_cluster_2`) with collision boxes.
  - Rotten timber stakes (`rotten_timber_stake_1`, `2`), rotten wood plank fragments (`rotten_wood_plank_fragment_1`, `2`), and submerged driftwood twig clusters (`driftwood_twig_cluster`).

---

## 7. Operations Manual: Complete Step-by-Step Execution Guide

### Step 7.1: Workspace Environment Setup
In every new terminal, source the environment using the pre-configured alias:
```bash
ws-isonavi
```
*(Equivalent manual command: `source /opt/ros/jazzy/setup.bash && source /home/prjwaal/isonavi_rov_ws/install/setup.bash`)*

### Step 7.2: Launch the 3D Simulation & Camera HUD
```bash
ros2 launch isonavi_rov_gazebo sim.launch.py
```
* **Gazebo 3D Simulation** and the **rqt_image_view Camera HUD** will automatically spawn side-by-side.
* **Launch Arguments Available:**
  * `world:=large_river` : Open river environment with submerged obstacles and currents (default: `large_river`).
  * `world:=indian_river`: High-turbidity river canyon environment.
  * `world:=pool_course` : Competition arena ($20 \times 10\,\text{m}$) with gate and targets.
  * `camera:=true/false` : Toggle automatic camera window popup (default: `true`).
  * `headless:=true/false`: Toggle Gazebo 3D GUI window (default: `false`).
  * `scattering:=1.6`    : Water turbidity attenuation factor (default: `1.6`).

### Step 7.3: Launch Keyboard Teleoperation
In a second terminal:
```bash
ws-isonavi
ros2 run isonavi_rov_ros teleop
```
**Keyboard Teleop Key Mappings:**

| Key | Motion | Description |
| :---: | :---: | :--- |
| `W` / `S` | **Surge** | Forward / Backward propulsion |
| `A` / `D` | **Sway** | Lateral Left / Right translation |
| `I` / `K` | **Heave** | Ascend / Descend (Vertical depth control) |
| `J` / `L` | **Yaw** | Turn Left (Port) / Turn Right (Starboard) |
| `Space` | **Hover** | Neutral throttle (Station-keeping holds position) |
| `1` - `4` | **Gears** | Speed scale ($25\%$, $50\%$, $75\%$, $100\%$ power) |

### Step 7.4: Dynamic River & Flood Control
In a third terminal:
```bash
ws-isonavi
ros2 run isonavi_rov_gazebo flood_ctl --preset moderate
```

### Step 7.5: Launch ROS 2 Computation Graph Visualizer
In a fourth terminal:
```bash
ws-isonavi
ros2 run rqt_graph rqt_graph
```

### Step 7.6: Real-Time Topic Telemetry Inspection
```bash
ws-isonavi
ros2 topic echo /depth            # Monitor vehicle depth in meters
ros2 topic echo /safety_state     # Monitor supervisor state machine
ros2 topic echo /thrusters        # Monitor 8 individual thruster forces (N)
```

### Step 7.7: Stopping the Simulation Cleanly
To kill all lingering Gazebo, ROS, and bridge tasks:
```bash
kill-sim
```

---

## 8. Complete Dependency Source Files Directory

Below is the definitive catalog of all source files in the ISONAVI-ROV project stack:

| Component | Absolute Path | Role & Key Functions |
| :--- | :--- | :--- |
| **Gazebo Launch** | [`sim.launch.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/launch/sim.launch.py) | Master launch description: launches Gazebo, ROS-GZ bridge, adapters, pilot, safety, vision, and camera viewer |
| **Flight Pilot Node** | [`pilot_node.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_ros/isonavi_rov_ros/pilot_node.py) | 6-DoF flight control, active dynamic positioning, damped PID depth/yaw hold, low-pass wrench smoothing |
| **Safety Supervisor** | [`safety_node.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_ros/isonavi_rov_ros/safety_node.py) | Supervisor state machine (`BOOT`, `DISARMED`, `ARMED`, `FAILSAFE`), tilt/depth boundary enforcement |
| **Matsya 7 Vision Node**| [`vision_node.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_ros/isonavi_rov_ros/vision_node.py) | Pyramidal Retinex, CLAHE, underwater white balance, YOLOv8 object detection, and spatial telemetry HUD |
| **Keyboard Teleop** | [`keyboard_teleop.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_ros/isonavi_rov_ros/keyboard_teleop.py) | Non-blocking terminal keyboard teleoperation node publishing 6-axis commands to `/joy` |
| **Flood Controller** | [`flood_controller.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/isonavi_rov_gazebo/flood_controller.py) | Dynamic river current, surge pulse, and turbidity simulation engine |
| **Flood CLI Tool** | [`flood_ctl.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/isonavi_rov_gazebo/flood_ctl.py) | Operator CLI to publish river presets and surge commands to `/flood` |
| **Thruster Demux** | [`thruster_bridge.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/isonavi_rov_gazebo/thruster_bridge.py) | Demultiplexes `/thrusters` (`Float64MultiArray`) to 8 individual joint thrust topics |
| **Depth Adapter** | [`depth_adapter.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/isonavi_rov_gazebo/depth_adapter.py) | Converts hydrostatic pressure and odometry ground-truth into standard `/depth` |
| **Tether Drag Model** | [`tether_drag.py`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/isonavi_rov_gazebo/tether_drag.py) | Cross-flow Morison drag model for trailing tether lengths up to $50\,\text{m}$ |
| **Bridge Parameter Map**| [`ros_gz_bridge.yaml`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/config/ros_gz_bridge.yaml) | 14-channel bidirectional topic mapping between Gazebo Harmonic and ROS 2 Jazzy |
| **ROV Physics & SDF** | [`model.sdf`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_description/models/isonavi_rov/model.sdf) | SDFormat 1.8 model: DART fluid added mass, Fossen damping, 8 thruster plugins, camera, IMU, spotlights |
| **Large River World** | [`large_river.sdf`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/worlds/large_river.sdf) | 3D river environment with bathymetry, submerged tree trunks, obstacles, and current layers |
| **Indian River World** | [`indian_river.sdf`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/worlds/indian_river.sdf) | River environment with high suspended particulate matter and optical turbidity |
| **Pool Course World** | [`pool_course.sdf`](file:///home/prjwaal/isonavi/isonavi-rov/ros/isonavi_rov_gazebo/worlds/pool_course.sdf) | $20 \times 10\,\text{m}$ competition pool course with graded buoyancy, gate, balloons, and obstacle maze |
| **Analytical Sim Core** | [`hydrodynamics.py`](file:///home/prjwaal/isonavi/isonavi-rov/simulation/isonavi/hydrodynamics.py) | Fossen equations of motion, 6x6 mass and damping matrices, cross-coupling terms |
| **Thrust Allocator** | [`allocator.py`](file:///home/prjwaal/isonavi/isonavi-rov/simulation/isonavi/allocator.py) | Moore-Penrose pseudo-inverse allocation matrix mapping 6-DoF wrench to 8 thruster forces |
| **Automated Sim Test** | [`test_sim.sh`](file:///home/prjwaal/isonavi/isonavi-rov/ros/test_sim.sh) | Automated end-to-end simulation test suite verifying build, bridge, teleop, and link-loss failsafe |
| **Node Chain Test** | [`test_chain.sh`](file:///home/prjwaal/isonavi/isonavi-rov/ros/test_chain.sh) | Integration test verifying safety gating and corridor centring logic |

---

## 9. ROS 2 Topic & Interface Specification

| Topic | Message Type | Publisher Node | Subscriber Node(s) | Update Rate | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/joy` | `sensor_msgs/msg/Joy` | `keyboard_teleop` | `pilot_node` | $10\,\text{Hz}$ | Pilot 6-axis analog command vector |
| `/wrench` | `geometry_msgs/msg/Wrench` | `pilot_node` | `safety_node` | $50\,\text{Hz}$ | Unconstrained 6-DoF pilot wrench (Forces & Torques) |
| `/thrusters` | `std_msgs/msg/Float64MultiArray` | `safety_node` | `thruster_bridge` | $50\,\text{Hz}$ | Power-governed, safety-gated 8-thruster force vector ($N$) |
| `/model/isonavi_rov/joint/thruster_{1..8}_joint/cmd_thrust` | `std_msgs/msg/Float64` | `thruster_bridge` | `parameter_bridge` | $50\,\text{Hz}$ | Individual thruster force forwarded to Gazebo |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `parameter_bridge` | `vision_node` | $30\,\text{Hz}$ | Raw $1280 \times 720$ RGB optical video stream |
| `/camera/image_detections`| `sensor_msgs/msg/Image` | `vision_node` | `rqt_image_view` | $30\,\text{Hz}$ | Matsya 7 Retinex-enhanced stream with Spatial AI HUD |
| `/depth` | `std_msgs/msg/Float64` | `depth_adapter` | `pilot_node`, `safety_node`| $50\,\text{Hz}$ | Hydrostatic depth in meters ($z \ge 0.0\,\text{m}$) |
| `/imu` | `sensor_msgs/msg/Imu` | `parameter_bridge` | `pilot_node`, `safety_node`| $100\,\text{Hz}$ | 6-axis linear acceleration & angular velocity |
| `/model/isonavi_rov/odometry`| `nav_msgs/msg/Odometry`| `parameter_bridge` | `depth_adapter`, `pilot_node`| $50\,\text{Hz}$ | Full 6-DoF vehicle pose and twist |
| `/safety_state` | `std_msgs/msg/String` | `safety_node` | Operators / Loggers | $10\,\text{Hz}$ | State machine string (`ARMED`, `HOLD`, `FAILSAFE`) |
| `/flood` | `std_msgs/msg/String` | `flood_ctl` | `flood_controller` | Event-driven | Flood preset commands & flow adjustments |
| `/clock` | `rosgraph_msgs/msg/Clock` | `parameter_bridge` | All ROS 2 Nodes | Simulation Rate | Gazebo Harmonic synchronized simulation time |

---

## 10. Verification & Test Summary

The entire stack has been verified under continuous integration testing:
1. **Compilation:** Built via `colcon build --symlink-install` across all 3 packages (`isonavi_rov_description`, `isonavi_rov_gazebo`, `isonavi_rov_ros`) with 0 errors.
2. **Simulation Bringup:** Headless and 3D GUI simulation runs without Ogre or X11 segmentation faults.
3. **Teleoperation Stability:** Zero hull resonance or twitching when maneuvering through all 6 degrees of freedom.
4. **Failsafe Assertions:** Verified automatic thruster power cutoff ($0.0\,\text{N}$) upon teleop link-loss timeout ($>0.5\,\text{s}$).
5. **Computer Vision Performance:** Matsya 7 Pyramidal Retinex and YOLOv8 pipeline runs in real time, delivering clear object detection and telemetry visualization on `/camera/image_detections`.
