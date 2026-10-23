#!/usr/bin/env bash
# Build the package and check the pilot, safety and actuator chain end to end:
# live joystick gives non-neutral pulse widths; joystick silence gives HOLD and neutral.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="${WS:-$HOME/isonavi_rov_ws}"
source /opt/ros/jazzy/setup.bash
mkdir -p "$WS/src"
ln -sfn "$REPO/ros/isonavi_rov_ros" "$WS/src/isonavi_rov_ros"
cd "$WS" && colcon build --symlink-install --packages-select isonavi_rov_ros > /tmp/rov_build.log 2>&1 \
  || { tail -20 /tmp/rov_build.log; exit 1; }
source "$WS/install/setup.bash"
export PYTHONPATH="$REPO/simulation:$PYTHONPATH" ISONAVI_ROOT="$REPO"

ros2 run isonavi_rov_ros safety > /tmp/rov_safety.log 2>&1 &
ros2 run isonavi_rov_ros pilot > /tmp/rov_pilot.log 2>&1 &
ros2 run isonavi_rov_ros actuator > /tmp/rov_act.log 2>&1 &
ros2 topic pub -r 20 depth std_msgs/msg/Float64 "{data: 0.8}" > /dev/null &
ros2 topic pub -r 50 imu sensor_msgs/msg/Imu "{orientation: {w: 1.0}}" > /dev/null &
ros2 topic pub -r 20 joy sensor_msgs/msg/Joy "{axes: [0.0, 0.5, 0.0, 0.0, 0.0]}" > /dev/null &
JOY=$!
sleep 9
echo "live link:"
timeout 12 ros2 topic echo --once safety_state | head -1
timeout 12 ros2 topic echo --once pwm_us | head -12 | tr '\n' ' '; echo
kill $JOY
sleep 2
echo "after joystick silence:"
timeout 12 ros2 topic echo --once safety_state | head -1
timeout 12 ros2 topic echo --once pwm_us | head -12 | tr '\n' ' '; echo
kill $(jobs -p) 2>/dev/null || true
pkill -f isonavi_rov_ros/lib 2>/dev/null || true
sleep 1

# Corridor centring: auto mode, button pressed, vehicle 0.15 m left of centre.
# Expected sway: -k_lat * 0.15 = -8.25 N, back towards the centreline.
ros2 run isonavi_rov_ros pilot --ros-args -p mode:=auto > /tmp/rov_pilot2.log 2>&1 &
ros2 topic pub -r 20 depth std_msgs/msg/Float64 "{data: 0.8}" > /dev/null &
ros2 topic pub -r 50 imu sensor_msgs/msg/Imu "{orientation: {w: 1.0}}" > /dev/null &
ros2 topic pub -r 5 corridor_offset std_msgs/msg/Float64 "{data: 0.15}" > /dev/null &
ros2 topic pub -r 20 joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0], buttons: [1]}" > /dev/null &
sleep 9
echo "centring engaged, offset 0.15 m:"
timeout 12 ros2 topic echo --once wrench_cmd | grep -A3 force | tr '\n' ' '; echo
kill $(jobs -p) 2>/dev/null || true
pkill -f isonavi_rov_ros/lib 2>/dev/null || true
sleep 1

# Shared mode in murky water: the same offset, no sway stick, so the command is
# the camera's share of the centring law alone, w * -8.25 N.
ros2 run isonavi_rov_ros pilot --ros-args -p mode:=shared -p scattering:=1.0 > /tmp/rov_pilot3.log 2>&1 &
ros2 topic pub -r 20 depth std_msgs/msg/Float64 "{data: 0.8}" > /dev/null &
ros2 topic pub -r 50 imu sensor_msgs/msg/Imu "{orientation: {w: 1.0}}" > /dev/null &
ros2 topic pub -r 5 corridor_offset std_msgs/msg/Float64 "{data: 0.15}" > /dev/null &
ros2 topic pub -r 20 joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0], buttons: [1]}" > /dev/null &
sleep 9
python3 - <<'EOF'
from isonavi.pilot import PilotConfig, blend_weight, read_error
from isonavi.vision import load_errors
w = blend_weight(read_error(PilotConfig(), 1.5, 1.0), max(load_errors(1.0).walls.values()))
print(f"shared, murky, offset 0.15 m: expected sway {-55.0 * 0.15 * w:.2f} N (camera weight {w:.2f})")
EOF
grep -o "camera weight [0-9.]*" /tmp/rov_pilot3.log
timeout 12 ros2 topic echo --once wrench_cmd | grep -A3 force | tr '\n' ' '; echo
kill $(jobs -p) 2>/dev/null || true
pkill -f isonavi_rov_ros/lib 2>/dev/null || true
