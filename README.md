# ros-turtlebot3-pid-kalman-control

ROS (Noetic) project for controlling a TurtleBot3 in Gazebo using **PID control** with **noisy sensor readings** and **Kalman filtering** to improve state estimation.

## What’s included
- PID controller for motion control
- Simulated noisy sensors
- Kalman filter to smooth/estimate measurements
- Gazebo simulation (TurtleBot3)

## Repository structure
Typical layout:
- `src/` — ROS package source code (nodes, launch files, config)
- `report/` (or similar) — assignment/report PDF and supporting material
- `README.md` — this file

If your repo differs, check the top-level folders on GitHub and start in `src/`.

## Report
The write-up/report is included in the repository. You can view it directly on GitHub:
https://github.com/jackmillington/ros-turtlebot3-pid-kalman-control

Tip: click the PDF file in the repo to preview it in the browser, or download it from there.

## Requirements
- Ubuntu + ROS Noetic
- Gazebo
- TurtleBot3 packages (and model setup)

## Build
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
````

## Run

(Replace package/node names below with the ones in this repo.)

```bash
roslaunch <package_name> <launch_file>.launch
# or
rosrun <package_name> <node_name>
```

## Notes

* This repo is intended to be used inside a `catkin_ws/src` workspace.
* Do not commit ROS build artifacts (`build/`, `devel/`, `logs/`).

## Author

Jack Millington

```
```
