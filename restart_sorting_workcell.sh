#!/usr/bin/env bash
# Rebuild and restart this workspace's Gazebo + ROS sorting stack.
# Keep the existing rosbridge websocket and web dashboard server alive.
set -e

source /opt/ros/jazzy/setup.bash
cd "$HOME/robotics_ws"

# Stop stale simulation, task/perception nodes, and Gazebo-to-ROS bridges.
pkill -TERM -f '[r]os2 launch robot_description gazebo.launch.py' 2>/dev/null || true
pkill -TERM -f '[g]z sim' 2>/dev/null || true
pkill -TERM -f '[r]os2 launch sorting_intelligence sorting_pipeline.launch.py' 2>/dev/null || true
pkill -TERM -f '[p]erception_node' 2>/dev/null || true
pkill -TERM -f '[t]ask_manager' 2>/dev/null || true
pkill -TERM -f '[p]arameter_bridge' 2>/dev/null || true
sleep 3

colcon build --symlink-install --packages-select robot_description sorting_intelligence
source install/setup.bash

stamp="$(date +%Y%m%d_%H%M%S)"
log_dir="$HOME/robotics_ws/logs/workcell_$stamp"
mkdir -p "$log_dir"

nohup ros2 launch robot_description gazebo.launch.py \
  >"$log_dir/gazebo.log" 2>&1 </dev/null &
echo "Gazebo launch PID: $!"

# Give Gazebo time to create its world and camera sensor.
sleep 15
nohup ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image' \
  >"$log_dir/gz_ros_bridge.log" 2>&1 </dev/null &
echo "Clock/camera bridge PID: $!"

nohup ros2 launch sorting_intelligence sorting_pipeline.launch.py \
  >"$log_dir/sorting_pipeline.log" 2>&1 </dev/null &
echo "Perception/task manager PID: $!"
echo "Started. Logs: $log_dir"
echo "Refresh http://localhost:8000 after the dashboard reconnects."