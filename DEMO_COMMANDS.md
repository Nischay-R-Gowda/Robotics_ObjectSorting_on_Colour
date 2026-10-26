# Sorting Workcell Demo Commands

Run these in **three separate Ubuntu WSL terminals**. Keep Terminal 1 and Terminal 2 open.

## Terminal 1 - rosbridge

\`\`\`bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
\`\`\`

## Terminal 2 - web dashboard

\`\`\`bash
cd ~/robotics_ws
python3 -m http.server 8000 --directory web --bind 0.0.0.0
\`\`\`

Open http://localhost:8000 in Windows Chrome.

## Terminal 3 - choose a color and start/restart the workcell

Choose one color per run: red, blue, or green.

\`\`\`bash
source /opt/ros/jazzy/setup.bash
bash ~/robotics_ws/src/robot_description/worlds/switch_demo_cube.sh red
bash ~/robotics_ws/restart_sorting_workcell.sh
\`\`\`

Replace red with blue or green to change the demo object. The restart helper rebuilds the project packages and launches Gazebo, the camera/clock bridge, perception, and task manager. It leaves rosbridge and the web server running.

## Run one cycle

1. Refresh http://localhost:8000.
2. Click **Connect**. Wait for the rosbridge connection, camera frame, and joint positions.
3. Click **Start** once and allow the cycle to finish.
4. Confirm the object is inside the matching color bin and the arm returns to its home pose.

To demonstrate another color, run the color switcher and restart helper again from Terminal 3, then refresh the dashboard.

## Quick camera and topic checks - another terminal

\`\`\`bash
source /opt/ros/jazzy/setup.bash
source ~/robotics_ws/install/setup.bash
ros2 topic list | grep -E '/camera/image_raw|/joint_states|/detected_object|/sorting/status'
ros2 topic hz /camera/image_raw
\`\`\`

The camera topic should have a non-zero rate. Stop the rate command with **Ctrl+C**. In the dashboard, ensure the object is inside the dashed pickup region.

## Safety and demo note

The dashboard's software E-stop is a simulation command, not a safety-rated hardware stop. Keep the Gazebo simulation unloaded when demonstrating stop behavior. Green and blue routes are configured in source; verify their placement in the actual Gazebo run before presenting them as successful.
