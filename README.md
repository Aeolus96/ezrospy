# EzRosPy

This ROS package simplifies writing scripted behaviors for ROS based Robots, particularly useful when you want to trigger or automate a set of sequential but still dynamic and repeatable behaviors. Additionally, it simplifies the setup process for simulation and real life testing.

The inspiration behind this package was replicating an Autonomous car's behaviour in simulation and in real life testing for customized scenarios. This allows for ease of testing and comparison when developing new software or trying out different hardware, for example regression testing between different Lidar scanners or obstacle avoidance algorithms.

The overarching goal of this package is to simplify ROS interfaces for short-term developers who may not need to build a full ROS stack or learn ROS in-depth but still use a ROS based robotic system for research or educational purposes. New developers or student teams using this package in conjunction with the ROS basics tutorials can speed up their development process for smaller projects.

## Installation

Prerequisites:

- Python 3.8 or higher
- ROS Noetic installed
- ROS workspace created with `catkin build`

```bash

# Go to the src folder of your ROS workspace

git clone https://github.com/Aeolus96/ezrospy.git

cd ezrospy

# Install dependencies
sudo chmod +x install.sh
./install.sh

```
