# ros-turtlebot3-pid-kalman-control

ROS (Noetic) project for TurtleBot3 control in Gazebo, covering PID control and state estimation with noisy sensor data (Kalman filtering), submitted for 3806ICT Robotics.

## Repository contents
- `assignment1_setup/` – ROS workspace/package files for the assignment.
- `assignment1_setup.tar` – Packaged version of the setup directory.
- `3806ICT_Jack_Millington_s5405915_assignment1.pdf` – Full report.
- `instructions.txt` – Setup/run notes.
- `README.md` – Project overview.

## Report
Open `3806ICT_Jack_Millington_s5405915_assignment1.pdf` in the repo to view the full write-up.

## Requirements
- Ubuntu with ROS Noetic
- Gazebo and TurtleBot3 packages

## Build (catkin)
Place `assignment1_setup/` inside your catkin workspace and build:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
Run
```
Follow instructions.txt for the exact launch/run commands used for the submission.


Author
Jack Millington
