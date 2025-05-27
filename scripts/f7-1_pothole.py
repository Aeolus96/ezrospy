#!/usr/bin/env python3


# 1. Test Goal
  # This test is intended to evaluate Ego vehicle’s ability to detect a pothole and safely change lane.
# 2. Test Setup
  # The following items shall be placed on the road:
  # - Barrel 1 to indicate a starting point at which vehicle is
  # stationary 
  # - Pothole (2 feet diameter solid white circle or
  # plastic mirror) 
  # - Barrel 2 to indicate an ending point
# 3. Test Script
  # 1. Begin test run
  # 2. Judge pushes 'start' button
  # 3. Vehicle takes off from full stop at Barrel 1
  # 4. Vehicle maintains the target speed (between 4 – 5 mph)
  # 5. Vehicle detects pothole and safely moves into the next lane
  # 6. Vehicle maintains the target speed in the new lane (between 4 – 5 mph)
  # 7. Vehicle reaches full stop within 3 ft from the Barrel 2
  # 8. End test run
# 4. Evaluation
  # Fail Criteria – run over the pothole
  # Penalties - hits barrel at the end of the run (25 points), stops further or closer than 3 ft to the Barrel
  # 2 (10 points)


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

# TODO: Add your code from here

actor.print_title("F7.1 Pothole Detection")

actor.print_highlights("Go Forward")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.drive_for(speed=1, angle=actor.lane_center, end_function=actor.yolo_look_for("pothole", 100))

actor.drive_for(speed=1, angle=1, speed_distance=3)

actor.drive_for(speed=1, angle=-1, speed_distance=5)

actor.drive_for(speed=1, angle=1, speed_distance=2)

actor.drive_for(speed=1, angle=actor.lane_center, end_function=actor.lidar_detect(lidar_zone=0, max_distance=3.0))

actor.stop_vehicle(duration=5.0)

actor.print_highlights("Pothole Complete!")
