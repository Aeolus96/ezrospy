#!/usr/bin/env python3


# Test FV4. Obstacle detection. Lane Changing
# 1. Test Goal
  # This test evaluates Ego vehicle’s ability to safely change lane if a stationary object is present within a current
  # lane.
# 2. Test Setup
  # The following items shall be placed on the road:
  # - Barrel 1 to indicate a starting point at which vehicle is
  # stationary 
  # - Barrel 2 to indicate obstacle 
  # - Barrel 3 to
  # indicate an ending point
# 3. Test Script
  # 1. Begin test run
  # 2. Judge pushes 'start' button
  # 3. Vehicle takes off from full stop at Barrel 1
  # 4. Vehicle maintains the target speed (between 3 – 5 mph)
  # 5. Vehicle detects obstacle (Barrel 2) and safely moves into the next lane
  # 6. Vehicle maintains the target speed in the new lane (between 3 – 5 mph)
  # 7. Vehicle reaches full stop within 3 ft from the obstacle (Barrel 3)
  # 8. End test run
# 4. Evaluation
  # Fail Criteria –hits Barrel 2, crosses white solid line
  # Penalties – hits Barrel 3 at the end of the run (25 points), lane change completed closer than 10 feet from the
  # obstacle (10 points)


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

# TODO: Add your code from here
