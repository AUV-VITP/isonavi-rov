# Project Merge Report: ISONAVI-ROV 3D Simulation Stack
**ROS 2 Jazzy Jalisco + Gazebo Harmonic (gz-sim 8.15.0) on Ubuntu 24.04.5 LTS**

---

## 1. Executive Summary & Purpose

This merge document outlines the transition of the **ISONAVI-ROV** project from standalone 2D/analytical Python simulations into an end-to-end, physics-accurate 3D digital twin running on **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic (gz-sim 8.15.0)** under **Ubuntu 24.04.5 LTS (Noble Numbat)**.

All development has been focused exclusively on the **`isonavi-rov`** open-frame inspection vehicle:
* **Vehicle Architecture:** $10.490\,\text{kg}$ dry mass, $66 \times 44 \times 33\,\text{cm}$ envelope, 8-thruster vectored configuration (4 horizontal at $\pm 45^\circ$, 4 vertical vectored).
* **Hydrostatic Stability:** Center of Gravity $\mathbf{r}_g = [0, 0, -0.0410]\,\text{m}$, Center of Buoyancy $\mathbf{r}_b = [0, 0, +0.0040]\,\text{m}$, ensuring a metacentric restoring height $BG = 45\,\text{mm}$ and $+2.00\,\text{N}$ net positive buoyancy ($V = 10.70\,\text{L}$).
* **Dynamic Simulation:** 6-DOF Fossen hydrodynamic formulations, implicit fluid added mass via DART, SNAME 1950 damping derivatives, 8 ducted thrusters, wide-angle optics under dual spotlights, 6-axis IMU, and hydrostatic depth sensing.
* **Competition Arena:** A full 3D pool course ($20 \times 10\,\text{m}$, sloping bottom from $0.6\,\text{m}$ to $1.8\,\text{m}$, submerged $1 \times 1\,\text{m}$ PVC competition gate, 4 target balloons, and a 3-section obstacle maze).
* **Failsafe Teleoperation:** Bi-directional ROS 2 Gazebo parameter bridge, pilot joystick teleoperation, automatic thruster force allocation, power limiting, and tether link-loss timeout failsafe.

---

## 2. System Requirements & Environment

### 2.1 Host Environment
* **Operating System:** Ubuntu 24.04.5 LTS (Noble Numbat), Linux x86_64 (`kernel 6.8.0+`)
* **ROS 2 Distribution:** ROS 2 Jazzy Jalisco (`ros-jazzy-desktop`, `ros-jazzy-ros-gz`)
* **Gazebo Simulator:** Gazebo Harmonic (`gz-sim 8.15.0`, `gz-tools2`, `gz-physics7`, `gz-sensors8`, `gz-rendering8`)
* **Physics Engine:** DART physics engine (v6.13) with ODE collision detection backend
* **Python Runtime:** Python 3.12 with `setuptools >= 68.0`, `numpy`, `scipy`, `opencv-python`, `pyyaml`

### 2.2 System Packages Required
```bash
sudo apt-get update && sudo apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-actuator-msgs \
    python3-colcon-common-extensions \
    python3-setuptools
```

### 2.3 Required Environment Variables
To ensure Gazebo discovers the robot models and ROS 2 locates the Python simulation modules, set the following environment variables (from the root of the cloned `isonavi-rov` repository):
```bash
# Navigate to the repository root directory
cd <path-to-isonavi-rov>
export REPO_DIR="$(pwd)"
export WS_DIR="$HOME/isonavi_rov_ws"

# Source ROS 2 Jazzy and workspace
source /opt/ros/jazzy/setup.bash
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"

# Export simulation paths
export PYTHONPATH="$REPO_DIR/simulation:$PYTHONPATH"
export ISONAVI_ROOT="$REPO_DIR"
export GZ_SIM_RESOURCE_PATH="$REPO_DIR/ros/isonavi_rov_description/models:$GZ_SIM_RESOURCE_PATH"
```

---

## 3. Detailed Inventory of Changes & Files

```
ros/
├── isonavi_rov_description/               # [NEW] Robot kinematics, URDF, and SDFormat model
│   ├── models/
│   │   └── isonavi_rov/
│   │       ├── model.config               # Gazebo Harmonic metadata
│   │       └── model.sdf                  # Complete 6-DOF SDFormat 1.8 vehicle model
│   ├── urdf/
│   │   └── isonavi_rov.urdf.xacro         # Kinematic tree for TF2 / RViz visualization
│   ├── package.xml                        # Ament package specification
│   ├── setup.py                           # Python package setup & asset export
│   └── setup.cfg                          # Executable script installation directory directive
├── isonavi_rov_gazebo/                    # [NEW] Simulation worlds, bridge configs & adapters
│   ├── worlds/
│   │   └── pool_course.sdf                # 3D pool arena with graded buoyancy and obstacle course
│   ├── config/
│   │   └── ros_gz_bridge.yaml             # Topic bridge parameter map (14 channels)
│   ├── isonavi_rov_gazebo/
│   │   ├── __init__.py
│   │   ├── thruster_bridge.py             # Demultiplexes Float64MultiArray to 8 joint topics
│   │   ├── depth_adapter.py               # Adapts fluid pressure/odometry into /depth
│   │   └── tether_drag.py                 # Cross-flow trailing tether hydrodynamic drag
│   ├── launch/
│   │   └── sim.launch.py                  # Master launch file for simulation stack
│   ├── package.xml                        # Ament package specification
│   ├── setup.py                           # Python package setup
│   └── setup.cfg                          # Executable script installation directory directive
├── isonavi_rov_ros/                       # Existing ROS 2 control nodes
│   ├── isonavi_rov_ros/
│   │   ├── safety_node.py                 # Supervisor state machine and power governor
│   │   ├── pilot_node.py                  # Joystick mapping, assist hold, shared vision blending
│   │   ├── vision_node.py                 # Optical target finder and corridor estimator
│   │   └── actuator_node.py               # PWM conversion for bench testing
│   └── setup.cfg
├── test_chain.sh                          # Existing ROS node logic integration test
└── test_sim.sh                            # [NEW] Automated end-to-end 3D simulation test suite
```

### 3.1 Detailed Description of Files Added/Modified

#### A. `ros/isonavi_rov_description/models/isonavi_rov/model.sdf`
The definitive SDFormat 1.8 description of the ISONAVI-ROV:
* **Base Link & Inertials:** Mass set to $10.490\,\text{kg}$; center of mass at $[0.0, 0.0, -0.0410]\,\text{m}$; principal moments $I_{xx} = 0.126\,\text{kg}\cdot\text{m}^2$, $I_{yy} = 0.288\,\text{kg}\cdot\text{m}^2$, $I_{zz} = 0.334\,\text{kg}\cdot\text{m}^2$.
* **Native DART `<fluid_added_mass>`:** Diagonal added mass terms ($xx=5.01\,\text{kg}$, $yy=5.61\,\text{kg}$, $zz=14.71\,\text{kg}$, $pp=0.125\,\text{kg}\cdot\text{m}^2$, $qq=0.089\,\text{kg}\cdot\text{m}^2$, $rr=0.146\,\text{kg}\cdot\text{m}^2$) embedded directly in `<inertial>`.
* **Calibrated Collision Geometries:** Four analytical shapes (`hull_collision`, `foam_collision`, `lower_chassis_collision`, `probe_collision`) precisely sized to displace $10.70\,\text{L}$ ($0.01070\,\text{m}^3$) with volumetric center at $[0.0, 0.0, +0.0040]\,\text{m}$, creating $+2.00\,\text{N}$ net upward buoyant force.
* **8 Vectored Thrusters:** Modeled with realistic hub visuals, revolute joints with viscous damping ($0.05\,\text{N}\cdot\text{s/rad}$) to avoid fixed-point precision overflows, and `gz-sim-thruster-system` plugins configured for $76\,\text{mm}$ propellers.
* **Hydrodynamics Plugin:** Configured with `gz-sim-hydrodynamics-system` using `<disable_added_mass>true</disable_added_mass>` (allowing DART to handle added mass natively without numerical instability) and Fossen negative damping derivatives.
* **Sensors:** Wide-angle RGB camera ($80^\circ$ FOV, $1280 \times 720$, $30\,\text{Hz}$), dual forward spotlights ($15\,\text{W}$, $1200\,\text{lm}$ each), 6-axis IMU ($100\,\text{Hz}$), and hydrostatic pressure sensor ($50\,\text{Hz}$).

#### B. `ros/isonavi_rov_gazebo/worlds/pool_course.sdf`
The complete competition and evaluation world:
* **Pool Basin:** $20\,\text{m}$ length, $10\,\text{m}$ width, $0.6\,\text{m}$ shallow end sloping smoothly to $1.8\,\text{m}$ deep end.
* **Graded Buoyancy Plugin:** `gz-sim-buoyancy-system` with `<graded_buoyancy>` layer at $z = 0.0\,\text{m}$. Below $z = 0.0\,\text{m}$, fluid density is $1000.0\,\text{kg/m}^3$ (water); above $z = 0.0\,\text{m}$, density transitions to $1.2\,\text{kg/m}^3$ (air), establishing a natural surface waterline.
* **Submerged Gate:** $1.0 \times 1.0\,\text{m}$ internal clearance PVC structure at $x = 5.0\,\text{m}$ submerged at depth $0.9\,\text{m}$.
* **Target Balloons:** 4 colored spheres (red, green, blue, yellow, diameter $0.20\,\text{m}$) moored at $x = 8.0\,\text{m}$ at differing depths.
* **3-Section Obstacle Slalom:** Dual wall corridors and offset columns testing lateral obstacle avoidance.
* **Odometry System:** Subscribes to ground truth state and publishes nav_msgs/Odometry at $50\,\text{Hz}$.

#### C. `ros/isonavi_rov_gazebo/config/ros_gz_bridge.yaml`
Configures `ros_gz_bridge parameter_bridge` across 14 channels:
1. `clock` (`gz.msgs.Clock` $\leftrightarrow$ `rosgraph_msgs/msg/Clock`)
2. `camera/image_raw` (`gz.msgs.Image` $\rightarrow$ `sensor_msgs/msg/Image`)
3. `camera/camera_info` (`gz.msgs.CameraInfo` $\rightarrow$ `sensor_msgs/msg/CameraInfo`)
4. `imu` (`gz.msgs.IMU` $\rightarrow$ `sensor_msgs/msg/Imu`)
5. `fluid_pressure` (`gz.msgs.FluidPressure` $\rightarrow$ `sensor_msgs/msg/FluidPressure`)
6. `model/isonavi_rov/odometry` (`gz.msgs.Odometry` $\rightarrow$ `nav_msgs/msg/Odometry`)
7. `model/isonavi_rov/joint/thruster_1..8_joint/cmd_thrust` (`std_msgs/msg/Float64` $\rightarrow$ `gz.msgs.Double`)

#### D. Adapters in `ros/isonavi_rov_gazebo/`
* **`thruster_bridge.py`:** Receives `/thrusters` (`std_msgs/msg/Float64MultiArray`) emitted by `safety_node` (containing allocated forces for thrusters 1 through 8 in Newtons) and maps them to individual `/model/isonavi_rov/joint/thruster_{i}_joint/cmd_thrust` topics.
* **`depth_adapter.py`:** Reads `/fluid_pressure` or odometry z-coordinate and publishes standard `/depth` (`std_msgs/msg/Float64`) for vehicle controllers.
* **`tether_drag.py`:** Calculates Morison drag $F_d = \frac{1}{2} \rho C_d d L v_{rel} |v_{rel}|$ for trailing tether lengths up to $50\,\text{m}$.

#### E. `ros/isonavi_rov_gazebo/launch/sim.launch.py`
Integrated launch system accepting launch arguments:
* `headless` (default: `true`): runs Gazebo server without GUI overhead, or launches GUI client when `false`.
* `mode` (default: `assist`): controller mode (`manual`, `assist`, `auto`, `shared`).
* `scattering` (default: `0.4`): water turbidity attenuation parameter $b$ ($\text{m}^{-1}$).
Manages process lifecycle for Gazebo Harmonic, parameter bridge, thruster adapter, depth adapter, safety supervisor, pilot node, and vision node.

#### F. `ros/test_sim.sh`
Automated bash verification script validating:
1. Compilation of all 3 ROS 2 packages (`colcon build --symlink-install`).
2. Gazebo Harmonic startup and environment discovery.
3. Bridge communication and sensor streaming (`/clock`, `/imu`, `/model/isonavi_rov/odometry`, `/depth`).
4. Active teleoperation: streams 10 Hz `/joy` commands, verifies `ARMED` supervisor state, verifies 8-thruster allocation matrix output, and confirms forwarding to Gazebo joint commands.
5. Failsafe link-loss verification: terminates joy stream, confirms timeout transition to `HOLD: pilot link lost`, and asserts all 8 thruster outputs are held at neutral ($0.0\,\text{N}$).
6. Safe process cleanup.

---

## 4. Deep-Dive Engineering & Root-Cause Analyses

During development and simulation bring-up on Ubuntu 24.04 (Noble) and Gazebo Harmonic, several critical engineering hurdles were diagnosed and resolved:

### 4.1 Explicit Added Mass Numerical Instability in DART
* **Observation:** Running `gz sim` with `gz-sim-hydrodynamics-system` caused ODE to abort within 20 to 50 simulation steps with:
  `ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact && aabbBound < dMaxIntExact" failed in collide()`.
* **Root Cause:** In `gz-sim-hydrodynamics-system`, added mass is implemented via explicit backward-difference numerical differentiation:
  $$\mathbf{a}_k = \frac{\mathbf{v}_k - \mathbf{v}_{k-1}}{\Delta t}, \quad \mathbf{F}_{am} = -\mathbf{M}_A \mathbf{a}_k$$
  In DART's forward step, external forces accelerate the body: $\mathbf{a}_k = \mathbf{M}^{-1} \mathbf{F}_{ext}$. When added mass exceeds dry mass (for ISONAVI-ROV heave, $M_{A,zz} = 14.71\,\text{kg} > M = 10.49\,\text{kg}$), this explicit loop produces an amplification factor $|1 - M_A / M| > 1$. The numerical oscillation magnifies exponentially at each step until the coordinates overflow fixed-point integer bounds in ODE.
* **Solution:** 
  1. We configured SDFormat 1.8 `<fluid_added_mass>` directly inside `<link name="base_link"><inertial>`. DART natively supports `<fluid_added_mass>`, including it implicitly on the left-hand side of the equations of motion:
     $$(\mathbf{M} + \mathbf{M}_A) \ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} = \boldsymbol{\tau}$$
     This formulation is unconditionally numerically stable regardless of the magnitude of $M_A$.
  2. We added `<disable_added_mass>true</disable_added_mass>` to `gz-sim-hydrodynamics-system`, preventing duplicate explicit added mass while retaining hydrodynamic damping and ocean current corrections.

### 4.2 SNAME 1950 Hydrodynamic Damping Signs
* **Observation:** The hydrodynamics plugin caused the vehicle to accelerate uncontrollably when moving.
* **Root Cause:** By SNAME 1950 and Fossen conventions used in Gazebo Harmonic (`HydrodynamicsUtils.hh`), damping terms in the SDF element must be strictly **negative**:
  $$D_{ii} = - \text{linear}_{ii} - \text{quad}_{ii} |v_i|$$
  $$F_d = - D \cdot v$$
  Supplying positive numbers in `<xU>` or `<xUabsU>` caused the plugin to negate them twice, resulting in a positive force in the direction of velocity (positive energy feedback).
* **Solution:** All damping coefficients in `model.sdf` were configured with their exact negative signs (`<xU>-15.9</xU>`, `<zW>-38.3</zW>`, `<xUabsU>-163.6</xUabsU>`, etc.).

### 4.3 Unbounded Buoyancy Acceleration vs Graded Buoyancy
* **Observation:** In early tests with `<uniform_fluid_density>1000.0</uniform_fluid_density>`, the vehicle accelerated upward indefinitely into the sky after breaching the surface.
* **Root Cause:** A uniform fluid density applies Archimedes buoyancy everywhere throughout the entire simulation coordinate space. Since ISONAVI-ROV is engineered with $+2.00\,\text{N}$ net positive buoyancy, without an air interface it accelerated continuously upward into space until AABB overflowed.
* **Solution:** Replaced uniform fluid density in `pool_course.sdf` with `<graded_buoyancy>`:
  ```xml
  <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
    <graded_buoyancy>
      <default_density>1000.0</default_density>
      <density_change>
        <above_depth>0.0</above_depth>
        <density>1.2</density>
      </density_change>
    </graded_buoyancy>
    <enable>isonavi_rov::base_link</enable>
  </plugin>
  ```
  At $z > 0.0\,\text{m}$, the fluid density drops to air density ($1.2\,\text{kg/m}^3$), causing the buoyant force to drop to zero as the vehicle breaches the waterline. The vehicle now realistically bobs and rests at hydrostatic equilibrium at the surface.

### 4.4 Air Pressure Sensor Type in SDFormat 14
* **Observation:** Gazebo reported `Attempting to load a Sensor, but the provided sensor type is missing or invalid` when using `type="fluid_pressure"`.
* **Root Cause:** In modern Gazebo / SDFormat 14+, the sensor type for hydrostatic pressure measurement is `air_pressure` (which outputs `gz.msgs.FluidPressure`).
* **Solution:** Updated sensor declaration to `type="air_pressure"` with `<air_pressure>` parameters in `model.sdf`.

### 4.5 Python 3.12 Ament Console Scripts Installation
* **Observation:** ROS 2 launch failed with `package 'isonavi_rov_gazebo' found, but libexec directory ... does not exist`.
* **Root Cause:** Python 3.12 setuptools defaults to installing console scripts into `bin/` rather than `lib/<package>/` unless explicitly instructed via `setup.cfg`.
* **Solution:** Created `setup.cfg` in `isonavi_rov_gazebo` and `isonavi_rov_description`:
  ```ini
  [develop]
  script_dir=$base/lib/isonavi_rov_gazebo
  [install]
  install_scripts=$base/lib/isonavi_rov_gazebo
  ```

---

## 5. Complete Steps to Build, Launch & Test

### Step 5.1: Create Workspace and Symlink Packages
```bash
# Navigate to the root of the cloned repository
cd <path-to-isonavi-rov>
REPO_DIR="$(pwd)"
WS_DIR="$HOME/isonavi_rov_ws"

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Prepare ROS 2 workspace
mkdir -p "$WS_DIR/src"
cd "$WS_DIR/src"

# Symlink project packages into workspace src
ln -sfn "$REPO_DIR/ros/isonavi_rov_description" .
ln -sfn "$REPO_DIR/ros/isonavi_rov_gazebo" .
ln -sfn "$REPO_DIR/ros/isonavi_rov_ros" .
```

### Step 5.2: Build the Workspace
```bash
cd "$WS_DIR"
colcon build --symlink-install
source "$WS_DIR/install/setup.bash"
```

### Step 5.3: Set Environment Variables
```bash
export PYTHONPATH="$REPO_DIR/simulation:$PYTHONPATH"
export ISONAVI_ROOT="$REPO_DIR"
export GZ_SIM_RESOURCE_PATH="$REPO_DIR/ros/isonavi_rov_description/models:$GZ_SIM_RESOURCE_PATH"
```

### Step 5.4: Run the Automated Test Suites
From the repository root directory:
```bash
cd "$REPO_DIR"

# Automated end-to-end 3D Gazebo Harmonic simulation test:
bash ros/test_sim.sh

# ROS 2 node logic and corridor centring test:
bash ros/test_chain.sh
```

### Step 5.5: Interactive Headless or GUI Launch
* **Launch Headless (default):**
  ```bash
  ros2 launch isonavi_rov_gazebo sim.launch.py headless:=true
  ```
* **Launch with 3D Gazebo GUI:**
  ```bash
  ros2 launch isonavi_rov_gazebo sim.launch.py headless:=false
  ```
* **Pilot Teleoperation in another terminal:**
  ```bash
  # Forward surge command (0.5 throttle)
  ros2 topic pub -r 10 /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0]}'
  ```
* **Inspect Topics:**
  ```bash
  ros2 topic echo /depth
  ros2 topic echo /safety_state
  ros2 topic echo /thrusters
  ```

---

## 6. Recommendations & Future Enhancements

1. **Hydrodynamic Damping Cross-Coupling:**
   Currently, the damping matrix uses principal diagonal terms ($X_u, Y_v, Z_w, K_p, M_q, N_r$). Adding off-diagonal cross-coupling terms (e.g., pitch-surge $M_{\dot{u}}$, yaw-sway $N_{\dot{v}}$) obtained from towing tank or high-fidelity CFD will further refine asymmetric hull interactions.
2. **Dynamic Tether Multi-Link Physics:**
   While `tether_drag.py` models lumped drag on the vehicle, Gazebo Harmonic can simulate the physical tether as a chain of small revolute-jointed cylindrical links with buoyant foam markers. This will allow modeling tether snagging and entanglements in the obstacle maze.
3. **Sensor Noise & Underwater Optical Shaders:**
   Integrate OGRE2 underwater post-processing shaders into Gazebo Harmonic to emulate water turbidity, wavelength-dependent absorption, and backscatter directly in the GPU rendering pipeline.
4. **Hardware-in-the-Loop (HIL) Integration:**
   The flight computer and ESP32 actuator bridge (`hil/`) can be plugged directly into the `/thrusters` and `/joy` ROS topics to fly the physical electronics against the simulated Gazebo vehicle in real time.

---

## 7. Suggested Changes for `README.md`

Below are the recommended updates to incorporate into the root `README.md` file to document the new Gazebo Harmonic simulation stack:

### Proposed Section Additions for `README.md`:

```markdown
### 3D Gazebo Harmonic Simulation (ROS 2 Jazzy)

The project includes an end-to-end 3D digital twin of the `isonavi-rov` running in **Gazebo Harmonic (gz-sim 8.15.0)** with **ROS 2 Jazzy**:
* **6-DOF Hydrodynamics & Stability:** Native DART fluid added mass, Fossen damping derivatives, calibrated $BG = 45\,\text{mm}$ metacentric height, and $+2.00\,\text{N}$ net positive buoyancy in freshwater.
* **Pool Arena:** $20 \times 10\,\text{m}$ competition course with sloping floor ($0.6$ to $1.8\,\text{m}$), $1 \times 1\,\text{m}$ PVC gate, 4 target balloons, and 3-section maze.
* **Sensors & Bridge:** 14-channel bidirectional bridge streaming camera images, IMU, hydrostatic depth, odometry, and 8 individual thruster command topics.

#### Running the Simulation Stack:
```bash
# Automated end-to-end simulation verification (builds workspace, tests bridge, failsafe)
bash ros/test_sim.sh

# Interactive launch (headless or 3D GUI)
ros2 launch isonavi_rov_gazebo sim.launch.py headless:=false mode:=assist
```
```

### Table Update in `README.md` ("What is here"):
Add the following entries to the "What is here" table in `README.md`:

| Module / Package | Purpose |
| --- | --- |
| `ros/isonavi_rov_description/` | 6-DOF SDFormat 1.8 model and URDF of `isonavi-rov` with DART added mass, Fossen damping, thruster plugins, and sensors |
| `ros/isonavi_rov_gazebo/` | Gazebo Harmonic pool course world, ROS-GZ parameter bridge config, thruster demux bridge, depth adapter, and launch files |
| `ros/test_sim.sh` | Automated end-to-end test suite verifying Gazebo Harmonic simulation, bridge, joy streaming, and failsafe |
