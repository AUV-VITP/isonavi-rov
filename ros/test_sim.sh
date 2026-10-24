#!/usr/bin/env bash
# ==============================================================================
# Automated End-to-End Simulation Test for ISONAVI-ROV
# ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04 Noble Numbat)
# ==============================================================================
set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="${WS:-$HOME/isonavi_rov_ws}"

echo "========================================================================"
echo "Step 1: Setting up ROS 2 Jazzy Workspace at $WS"
echo "========================================================================"
source /opt/ros/jazzy/setup.bash
mkdir -p "$WS/src"
ln -sfn "$REPO/ros/isonavi_rov_ros" "$WS/src/isonavi_rov_ros"
ln -sfn "$REPO/ros/isonavi_rov_description" "$WS/src/isonavi_rov_description"
ln -sfn "$REPO/ros/isonavi_rov_gazebo" "$WS/src/isonavi_rov_gazebo"

cd "$WS"
colcon build --symlink-install > /tmp/sim_build.log 2>&1 || {
    echo "ERROR: Workspace build failed! Last 25 log lines:"
    tail -25 /tmp/sim_build.log
    exit 1
}
source "$WS/install/setup.bash"

export PYTHONPATH="$REPO/simulation:$PYTHONPATH"
export ISONAVI_ROOT="$REPO"
export GZ_SIM_RESOURCE_PATH="$REPO/ros/isonavi_rov_description/models:$WS/install/isonavi_rov_description/share/isonavi_rov_description/models:$GZ_SIM_RESOURCE_PATH"

cleanup() {
    echo "Cleaning up test processes..."
    kill $(jobs -p) 2>/dev/null || true
    pkill -f "ros2 launch isonavi_rov_gazebo" 2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    pkill -f "thruster_bridge" 2>/dev/null || true
    pkill -f "depth_adapter" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby.*gz" 2>/dev/null || true
    pkill -f "isonavi_rov_ros" 2>/dev/null || true
}
trap cleanup EXIT

echo "========================================================================"
echo "Step 2: Launching Gazebo Harmonic Pool Course Simulation (Headless)"
echo "========================================================================"
ros2 launch isonavi_rov_gazebo sim.launch.py headless:=true > /tmp/sim_launch.log 2>&1 &
LAUNCH_PID=$!

echo "Waiting for Gazebo and ROS nodes to initialize..."
sleep 6

# Verify simulation clock is ticking
echo "Verifying /clock topic..."
timeout 6 ros2 topic echo /clock --once > /tmp/test_clock.txt || {
    echo "ERROR: /clock topic timed out!"
    tail -30 /tmp/sim_launch.log
    exit 1
}
echo "Clock verified: $(cat /tmp/test_clock.txt | head -2)"

# Verify IMU sensor
echo "Verifying /imu topic..."
timeout 6 ros2 topic echo /imu --once > /tmp/test_imu.txt || {
    echo "ERROR: /imu topic timed out!"
    exit 1
}
echo "IMU verified: frame_id = $(grep frame_id /tmp/test_imu.txt | head -1 | awk '{print $2}')"

# Verify Odometry
echo "Verifying /model/isonavi_rov/odometry..."
timeout 6 ros2 topic echo /model/isonavi_rov/odometry --once > /tmp/test_odom.txt || {
    echo "ERROR: Odometry topic timed out!"
    exit 1
}
echo "Odometry verified: frame_id = $(grep frame_id /tmp/test_odom.txt | head -1 | awk '{print $2}')"

# Verify Depth Adapter
echo "Verifying /depth topic..."
timeout 6 ros2 topic echo /depth --once > /tmp/test_depth.txt || {
    echo "ERROR: Depth adapter /depth timed out!"
    exit 1
}
echo "Depth adapter verified: $(cat /tmp/test_depth.txt | head -1)"

echo "========================================================================"
echo "Step 3: Testing Active Pilot Control (Streaming Joy at 10 Hz)"
echo "========================================================================"
# Stream forward surge = 0.6
ros2 topic pub -r 10 /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.6, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0]}' > /dev/null 2>&1 &
JOY_PID=$!

sleep 2

# Check safety state is ARMED
echo "Checking /safety_state..."
timeout 4 ros2 topic echo /safety_state --once > /tmp/test_state.txt
echo "Safety state: $(cat /tmp/test_state.txt | head -1)"
grep -q "ARMED" /tmp/test_state.txt || {
    echo "ERROR: Safety state is not ARMED during active joy streaming!"
    exit 1
}

# Check thruster forces
echo "Checking /thrusters multi-array..."
timeout 4 ros2 topic echo /thrusters --once > /tmp/test_thrusters.txt
cat /tmp/test_thrusters.txt | grep -A9 data
grep -q "data:" /tmp/test_thrusters.txt || {
    echo "ERROR: No data on /thrusters!"
    exit 1
}

# Check individual thruster command forwarded to Gazebo
echo "Checking Gazebo bridge thruster 1 command..."
timeout 4 ros2 topic echo /model/isonavi_rov/joint/thruster_1_joint/cmd_thrust --once > /tmp/test_cmd.txt
echo "Joint 1 cmd_thrust: $(cat /tmp/test_cmd.txt | head -1)"
grep -q "data:" /tmp/test_cmd.txt || {
    echo "ERROR: Bridge failed to forward cmd_thrust to Gazebo!"
    exit 1
}

echo "========================================================================"
echo "Step 4: Testing Failsafe Link-Loss Detection"
echo "========================================================================"
echo "Killing joy stream to simulate tether/telemetry communication loss..."
kill $JOY_PID 2>/dev/null || true

# Comms timeout is 0.5s; wait 1.2s to trigger HOLD
sleep 1.2

echo "Checking safety state after silence..."
timeout 4 ros2 topic echo /safety_state --once > /tmp/test_failsafe_state.txt
echo "Failsafe safety state: $(cat /tmp/test_failsafe_state.txt | head -1)"
grep -q "HOLD" /tmp/test_failsafe_state.txt || {
    echo "ERROR: Supervisor failed to transition to HOLD!"
    exit 1
}

echo "Checking thruster forces under HOLD..."
timeout 4 ros2 topic echo /thrusters --once > /tmp/test_hold_thrusters.txt
cat /tmp/test_hold_thrusters.txt | grep -A9 data
# All 8 thrusters must be zeroed under HOLD
python3 -c "
with open('/tmp/test_hold_thrusters.txt') as f:
    text = f.read()
import re
vals = [float(x) for x in re.findall(r'-?\d+\.\d+', text)]
assert all(v == 0.0 for v in vals[:8]), f'Thrusters not zeroed under HOLD: {vals[:8]}'
print('SUCCESS: All 8 thrusters held neutral (0.0 N) under link loss!')
"

echo "========================================================================"
echo "ALL SIMULATION INTEGRATION TESTS PASSED SUCCESSFULLY!"
echo "========================================================================"
