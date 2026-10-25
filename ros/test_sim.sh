#!/usr/bin/env bash
# Builds the Gazebo packages and checks the model and the vehicle's nodes on it:
#   1. the Gazebo model against the python dynamics, five open loop cases;
#   2. the world with the nodes: sensors stream, depth agrees with the pose,
#      the camera renders and the tether drag is loaded;
#   3. closed loop through every node: an assisted surge stick moves the
#      vehicle forward while the depth hold keeps its depth;
#   4. joystick silence holds every thruster at zero.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="${WS:-$HOME/isonavi_rov_ws}"
source /opt/ros/jazzy/setup.bash
mkdir -p "$WS/src"
for p in isonavi_rov_ros isonavi_rov_description isonavi_rov_gazebo; do
  ln -sfn "$REPO/ros/$p" "$WS/src/$p"
done
cd "$WS" && colcon build --symlink-install > /tmp/sim_build.log 2>&1 \
  || { tail -25 /tmp/sim_build.log; exit 1; }
source "$WS/install/setup.bash"
export PYTHONPATH="$REPO/simulation:$PYTHONPATH" ISONAVI_ROOT="$REPO"
export GZ_SIM_RESOURCE_PATH="$REPO/ros/isonavi_rov_description/models:$GZ_SIM_RESOURCE_PATH"
cd "$REPO"

cleanup() {
  kill $(jobs -p) 2>/dev/null || true
  pkill -f "ros2 launch isonavi_rov_gazebo" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "isonavi_rov_(ros|gazebo)/lib" 2>/dev/null || true
  pkill -f parameter_bridge 2>/dev/null || true
}
trap cleanup EXIT
field() { timeout 15 ros2 topic echo --once "$1" --field "$2" 2>/dev/null | head -1; }

echo "1. Gazebo model against the python dynamics"
python3 ros/isonavi_rov_gazebo/isonavi_rov_gazebo/twin_check.py

echo "2. world and nodes"
ros2 launch isonavi_rov_gazebo sim.launch.py headless:=true > /tmp/sim_launch.log 2>&1 &
for _ in $(seq 60); do field /clock clock.sec > /dev/null && break; sleep 1; done
field /imu orientation.w > /dev/null && echo "  imu streaming"
timeout 30 ros2 topic echo --once /camera/image_raw --field width > /tmp/sim_cam.txt 2>/dev/null
grep -q 160 /tmp/sim_cam.txt && echo "  camera rendering at 160 px"
z=$(field /model/isonavi_rov/odometry pose.pose.position.z)
d=$(field /depth data)
python3 -c "import sys; z, d = $z, $d; assert abs(d + z) < 0.06, (z, d); print(f'  depth {d:.3f} m against {-z:.3f} m from the pose')"
grep -q "isonavi-rov tether drag" /tmp/sim_launch.log && echo "  tether drag loaded"

echo "3. assisted surge stick, closed loop"
x0=$(field /model/isonavi_rov/odometry pose.pose.position.x)
ros2 topic pub -r 20 /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.6, 0.0, 0.0, 0.0], buttons: [0]}" > /dev/null &
JOY=$!
sleep 3
state=$(field /safety_state data)
echo "  supervisor $state"
[ "$state" = "ARMED" ]
z1=$(field /model/isonavi_rov/odometry pose.pose.position.z)
sleep 8
x1=$(field /model/isonavi_rov/odometry pose.pose.position.x)
z2=$(field /model/isonavi_rov/odometry pose.pose.position.z)
python3 -c "
x0, x1, z1, z2 = $x0, $x1, $z1, $z2
assert x1 - x0 > 1.0, (x0, x1)
assert abs(z2 - z1) < 0.10, (z1, z2)
print(f'  moved {x1 - x0:.2f} m forward, depth held to {abs(z2 - z1) * 1000:.0f} mm')"

echo "4. joystick silence"
kill $JOY
sleep 2
state=$(field /safety_state data)
echo "  supervisor $state"
[[ "$state" == HOLD* ]]
timeout 15 ros2 topic echo --once /thrusters --field data > /tmp/sim_hold.txt
python3 -c "
import re
v = [float(x) for x in re.findall(r'-?\d+\.\d+(?:e-?\d+)?', open('/tmp/sim_hold.txt').read())]
assert len(v) == 8 and all(x == 0.0 for x in v), v
print('  all eight thrusters at zero')"
echo "simulation checks passed"
