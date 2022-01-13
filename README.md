# ROS2 ATR Interfaces

ROS2 Packages for the ATR communication interfaces

The documentation of the project can be found here:
<https://edeanl.github.io/ATR_Interfaces>

This repo is based on https://github.com/edeanl/ATR_Interfaces

## Description

This repository provides ros2 packages with the communication interfaces for the ATRs.

NOTE: Contains ComSat and ATR scheduler

__The simulation is not completely flawless due to bugs probably in the ATR scheduler node (ATR_scheduling/atr_examples_py/atr_examples_py/ATRScheduler.py). The problem is probably in the discrete path generation. The ATRs make sharp turns when the same coordinates are received recurrently and also there are deviations to the generated schedule from ComSat. For future work, these bugs should be solved and also a time display should be implemented in rviz to track the elapsed time since the simulation is not executed in real-time.__

The scenario includes the following modules:

1. ATR Formations
1. Global to Discrete Pose
1. Job-based ATR Scheduler
1. Discrete to Global Pose
1. RTSP Stream
1. ATR Tracker
1. Semantic Segmentation
1. NONA Generator
1. Dynamic Obstacle Prediction
1. ATR Trajectory Generator
1. ATR Fleet Control
1. ATR Control
1. ROS2-Redis Bridge

The ros packages have been tested in Ubuntu 20.04 with ROS Foxy.

It provides three ros packages:

- atr_interfaces: With the msg and srv definitions
- atr_examples: C++ implementations of the nodes for the ATR scenario using the atr_interfaces
- atr_examples_py: Some examples in python (needs more work)

## Usage

Copy the ros packages to your ros2 workspace:

```bash
cp atr_interfaces atr_examples /home/usr/ros_workspace/src/
```

Source your ros distro:

```bash
source /opt/ros/foxy/setup.bash
```

Compile your workspace, e.g.

```bash
cd /home/usr/ros_workspace/src/
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_STANDARD=14
```

There's a launch file (python) that will run all the nodes and visualization tools. In that python file, you can see which are the used nodes for this demo.

```bash
cd /home/usr/ros_workspace/src/
source install/setup.bash
ros2 launch atr_examples atr_interfaces_test_launch.py
```

Example usage of ATR scheduler:

Launch atr_interfaces_test_launch according to above instructions

In new terminal:
```bash
cd /home/usr/ros_workspace/src/
source install/setup.bash
ros2 run atr_examples_py scheduler 
```

In new terminal call service:
```bash
cd /home/usr/ros_workspace/src/
source install/setup.bash
ros2 service call /get_job_schedule atr_interfaces/srv/ComSat problem:\ \'Volvo_test_case_new_1\'\ 
```


## TODO

The documentation of the modules is not complete, for now, the links point to the same draft pdf file.

Missing modules:

- Global to Discrete Pose
- RTSP Stream
- ROS2-Redis Bridge
  
