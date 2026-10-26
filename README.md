# Intelligent Robotic Sorting Workcell

ROS 2 Jazzy and Gazebo Harmonic simulation of a camera-guided sorting arm with a browser monitoring interface.

## Features

- Gazebo sorting scene with a red object and red, blue, and green sorting bins.
- ROS camera bridge and color perception node.
- Task manager with configured red, blue, and green bin routes.
- Static browser dashboard served from web/, connected through rosbridge.
- Demo cube color switcher and workspace restart helper.

The arm uses configured joint targets for pickup and placement. Camera detections report image-pixel coordinates; they are not used for general camera-to-robot calibration. Validate each route in the target Gazebo setup before claiming a successful physical sort.

## Requirements

- Ubuntu 24.04 under WSL 2
- ROS 2 Jazzy
- Gazebo Harmonic and the ROS-Gazebo integration
- rosbridge_server

## Demo

Follow [DEMO_COMMANDS.md](DEMO_COMMANDS.md) for the terminal-by-terminal run sequence and color selection. The printable one-page command sheet is provided separately.

## Source layout

- src/robot_description/: robot model, controllers, launch files, and Gazebo world
- src/sorting_intelligence/: perception and task manager
- web/: browser monitoring interface
- restart_sorting_workcell.sh: rebuilds the two packages and restarts the simulation stack
- src/robot_description/worlds/switch_demo_cube.sh: changes the demo cube to red, blue, or green

This repository contains source files only. Colcon output, runtime logs, Python caches, and saved pre-edit backups are excluded.
