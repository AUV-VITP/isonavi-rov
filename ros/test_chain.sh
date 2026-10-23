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
sleep 6
echo "live link:"
timeout 5 ros2 topic echo --once safety_state | head -1
timeout 5 ros2 topic echo --once pwm_us | head -12 | tr '\n' ' '; echo
kill $JOY
sleep 2
echo "after joystick silence:"
timeout 5 ros2 topic echo --once safety_state | head -1
timeout 5 ros2 topic echo --once pwm_us | head -12 | tr '\n' ' '; echo
kill $(jobs -p) 2>/dev/null || true
pkill -f isonavi_rov_ros/lib 2>/dev/null || true
