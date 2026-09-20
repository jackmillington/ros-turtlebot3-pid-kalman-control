# TurtleBot3 PID + Kalman Control

A ROS Noetic and Gazebo robotics project demonstrating closed-loop motion control and state estimation under noisy sensor measurements.

The system simulates a TurtleBot3 Burger approaching a target while using an emulated multi-direction sonar array. Raw measurements are filtered with a Kalman estimator, then fed into a PID controller that drives the robot toward a fixed stand-off distance.

## What this project demonstrates

- ROS node, topic, service, message and launch-file design in C++
- PID feedback control for autonomous robot motion
- Kalman filtering for noisy distance measurements
- Sensor-noise characterisation using online variance estimation
- Covariance propagation from noisy position estimates
- Gazebo model-state integration
- Parameterised controller gains for repeatable tuning

## Architecture

```mermaid
flowchart LR
    G[Gazebo world] --> S[Emulated sonar array]
    G --> M[Noisy model-state service]
    S --> K[Kalman estimator]
    M --> K
    K --> P[PID controller]
    P --> C[/cmd_vel]
    C --> G
```

## Key components

| Component | Purpose |
| --- | --- |
| `src/sonars.cc` | Generates directional sonar-style measurements from the simulated environment |
| `src/noise.cc` | Adds Gaussian measurement noise |
| `src/model_state.cc` | Exposes noisy robot position and covariance through a ROS service |
| `src/sonar_variance_estimator.cpp` | Estimates measurement variance using Welford's online algorithm |
| `src/controller.cpp` | Performs Kalman predict/update steps and PID velocity control |
| `msg/Sonars.msg` | Custom six-sensor ROS message |
| `srv/ModelState.srv` | Custom noisy-position and covariance service |

## Stack

- C++
- ROS Noetic
- Gazebo
- TurtleBot3
- Catkin
- PID control
- Kalman filtering

## Build

Clone the repository into a ROS Noetic catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/jackmillington/ros-turtlebot3-pid-kalman-control.git

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run

The simulation expects Gazebo models named `turtlebot3_burger` and `unit_box`.

Start the sensor and estimation nodes:

```bash
rosrun turtlebot3_pid_kalman_control noisy_sonars
rosrun turtlebot3_pid_kalman_control model_state
```

Optionally estimate sonar measurement variance:

```bash
rosrun turtlebot3_pid_kalman_control sonar_variance_estimator
```

Launch the controller:

```bash
roslaunch turtlebot3_pid_kalman_control controller.launch
```

PID gains are exposed as private ROS parameters in `launch/controller.launch`.

## Scope

This repository targets simulation in ROS Noetic/Gazebo. It is intended to demonstrate the control and estimation stack clearly rather than provide a production hardware driver.

## License

GPL-3.0. Individual source files retain any existing third-party copyright and licence notices.
